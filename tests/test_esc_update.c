#define _POSIX_C_SOURCE 200809L
#include "check.h"
#include "flashimg_update.h"
#include "sha256.h"
#include "crc32.h"
#include <elf.h>
#include <stdarg.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>

#define SENSOR 0x20001000U
#define MCI 0x20001010U
#define TICK 0x20001020U
static char root[256], err[1024];
/* Host ELF symbols only: these are never run as a firmware model. */
uint32_t nucleo_image_kind, fwu_host, can_fwu_request, can_fwu_dev, can_fwu_result;
uint32_t fwu_test_image, fwu_test_image_size, can_fwu_enter_boot, can_fwu_boot_result;

static void path(char out[1024], const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt); int n = vsnprintf(out, 1024, fmt, ap); va_end(ap);
  CHECK(n >= 0 && n < 1024);
}

static void put(const char *name, const void *data, size_t n)
{
  FILE *f = fopen(name, "wb"); CHECK(f != NULL);
  CHECK_EQ(fwrite(data, 1, n, f), n); CHECK_EQ(fclose(f), 0);
}

static char *get(const char *name)
{
  uint8_t *data; size_t n;
  CHECK_EQ(flashimg_read_file(name, &data, &n), 0);
  char *s = realloc(data, n+1); CHECK(s != NULL); s[n] = 0;
  return s;
}

/* One load segment includes padding and a RAM-VMA section whose LMA is flash.
 * The symbol table describes only documented addresses used by the RAM guard. */
static void tiny_elf32(const char *name, bool symbols)
{
  uint8_t b[1024] = {0};
  Elf32_Ehdr e = {0};
  memcpy(e.e_ident, ELFMAG, SELFMAG);
  e.e_ident[EI_CLASS] = ELFCLASS32; e.e_ident[EI_DATA] = ELFDATA2LSB; e.e_ident[EI_VERSION] = EV_CURRENT;
  e.e_type = ET_EXEC; e.e_machine = EM_ARM; e.e_version = EV_CURRENT; e.e_ehsize = sizeof e;
  e.e_phoff = sizeof e; e.e_phnum = 1; e.e_phentsize = sizeof(Elf32_Phdr);
  e.e_shoff = 512; e.e_shnum = 5; e.e_shentsize = sizeof(Elf32_Shdr);
  Elf32_Phdr p = {.p_type=PT_LOAD, .p_offset=128, .p_paddr=0x08004000, .p_vaddr=0x20000000,
                 .p_filesz=128, .p_memsz=160};
  Elf32_Shdr sh[5] = {0};
  sh[1] = (Elf32_Shdr){.sh_type=SHT_PROGBITS, .sh_flags=SHF_ALLOC|SHF_WRITE, .sh_addr=0x20000020,
                      .sh_offset=160, .sh_size=4};
  sh[2] = (Elf32_Shdr){.sh_type=SHT_NOBITS, .sh_flags=SHF_ALLOC|SHF_WRITE, .sh_size=32};
  sh[3] = (Elf32_Shdr){.sh_type=SHT_SYMTAB, .sh_offset=256, .sh_size=symbols ? 5*sizeof(Elf32_Sym) : 0,
                      .sh_link=4, .sh_entsize=sizeof(Elf32_Sym)};
  sh[4] = (Elf32_Shdr){.sh_type=SHT_STRTAB, .sh_offset=384, .sh_size=64};
  const char names[] = "\0BusVoltageSensor_M1\0Mci\0uwTick\0drive_app\0";
  Elf32_Sym syms[5] = {0};
  syms[1] = (Elf32_Sym){.st_name=1, .st_value=SENSOR, .st_size=8, .st_shndx=1};
  syms[2] = (Elf32_Sym){.st_name=21, .st_value=MCI, .st_size=2, .st_shndx=1};
  syms[3] = (Elf32_Sym){.st_name=25, .st_value=TICK, .st_size=4, .st_shndx=1};
  syms[4] = (Elf32_Sym){.st_name=32, .st_value=TICK+4, .st_size=4, .st_shndx=1};
  /* Derive offsets rather than relying on spelling lengths. */
  const char *cursor = names+1;
  for (int i = 1; i < 5; i++) { syms[i].st_name = (Elf32_Word)(cursor-names); cursor += strlen(cursor)+1; }
  memcpy(b, &e, sizeof e); memcpy(b+e.e_phoff, &p, sizeof p);
  memcpy(b+e.e_shoff, sh, sizeof sh); memcpy(b+256, syms, sizeof syms); memcpy(b+384, names, sizeof names);
  b[160]=0x20; b[161]=0x21; b[162]=0x22; b[163]=0x23;
  put(name, b, sizeof b);
}

static void test_flash_matches_sections(void)
{
  char elf[1024]; path(elf, "%s/tiny.elf", root); tiny_elf32(elf, true);
  uint8_t *flash = malloc(FLASHIMG_FLASH_SIZE); CHECK(flash != NULL); memset(flash, 0xff, FLASHIMG_FLASH_SIZE);
  memcpy(flash+0x4020, "\x20\x21\x22\x23", 4);
  CHECK(flashimg_matches(elf, flash, FLASHIMG_FLASH_SIZE));
  /* A whole-segment comparison would reject the deliberately different padding. */
  flash[0x4000] = 0x55; CHECK(flashimg_matches(elf, flash, FLASHIMG_FLASH_SIZE));
  flash[0x4021] ^= 1; CHECK(!flashimg_matches(elf, flash, FLASHIMG_FLASH_SIZE)); flash[0x4021] ^= 1;
  CHECK(!flashimg_matches(elf, flash, 0x4023));
  uint8_t *b; size_t n; CHECK_EQ(flashimg_read_file(elf, &b, &n), 0);
  Elf32_Shdr sh; memcpy(&sh, b+512+sizeof sh, sizeof sh);
  sh.sh_flags = 0; memcpy(b+512+sizeof sh, &sh, sizeof sh); put(elf, b, n);
  CHECK(!flashimg_matches(elf, flash, FLASHIMG_FLASH_SIZE));
  sh.sh_flags = SHF_ALLOC; sh.sh_offset = 300; memcpy(b+512+sizeof sh, &sh, sizeof sh); put(elf, b, n);
  CHECK(!flashimg_matches(elf, flash, FLASHIMG_FLASH_SIZE));
  sh.sh_offset = 160; memcpy(b+512+sizeof sh, &sh, sizeof sh);
  Elf32_Phdr ph; memcpy(&ph, b+sizeof(Elf32_Ehdr), sizeof ph);
  ph.p_paddr = 0x07000000; memcpy(b+sizeof(Elf32_Ehdr), &ph, sizeof ph); put(elf, b, n);
  CHECK(!flashimg_matches(elf, flash, FLASHIMG_FLASH_SIZE));
  free(b); free(flash);
}

static void test_flash_matches_elf64(void)
{
  char elf[1024]; path(elf, "%s/tiny64.elf", root);
  uint8_t b[512] = {0}, flash[16] = {1,2,3,4};
  Elf64_Ehdr e = {0}; memcpy(e.e_ident, ELFMAG, SELFMAG);
  e.e_ident[EI_CLASS]=ELFCLASS64; e.e_ident[EI_DATA]=ELFDATA2LSB;
  e.e_phoff=sizeof e; e.e_phentsize=sizeof(Elf64_Phdr); e.e_phnum=1;
  e.e_shoff=256; e.e_shentsize=sizeof(Elf64_Shdr); e.e_shnum=2;
  Elf64_Phdr ph = {.p_type=PT_LOAD, .p_offset=128, .p_paddr=0x08000000, .p_filesz=4};
  Elf64_Shdr sh = {.sh_type=SHT_PROGBITS, .sh_flags=SHF_ALLOC, .sh_offset=128, .sh_size=4};
  memcpy(b, &e, sizeof e); memcpy(b+e.e_phoff, &ph, sizeof ph); memcpy(b+320, &sh, sizeof sh);
  memcpy(b+128, flash, 4); put(elf, b, sizeof b);
  CHECK(flashimg_matches(elf, flash, sizeof flash));
  flash[0]++; CHECK(!flashimg_matches(elf, flash, sizeof flash)); flash[0]--;
  ph.p_paddr = UINT64_MAX; memcpy(b+e.e_phoff, &ph, sizeof ph); put(elf, b, sizeof b);
  CHECK(!flashimg_matches(elf, flash, sizeof flash));
  e.e_shoff = UINT64_MAX-1; memcpy(b, &e, sizeof e); put(elf, b, sizeof b);
  CHECK(!flashimg_matches(elf, flash, sizeof flash));
  for (size_t len = 0; len < sizeof(Elf64_Ehdr); len++) {
    put(elf, b, len); CHECK(!flashimg_matches(elf, flash, sizeof flash));
  }
  put(elf, "not an ELF", 10); CHECK(!flashimg_matches(elf, flash, sizeof flash));
  CHECK(!flashimg_matches("/nonexistent/esc-update.elf", flash, sizeof flash));
}

static void test_installed_app_elf(void)
{
  char build[1024], backups[1024], dir[1024], a[1024], z[1024], known[1024], result[4096];
  path(build, "%s/build[glob]", root); CHECK_EQ(mkdir(build,0700),0);
  path(backups, "%s/backups", root); CHECK_EQ(mkdir(backups,0700),0);
  path(dir, "%s/esc", build); CHECK_EQ(mkdir(dir,0700),0);
  path(dir, "%s/esc/A", build); CHECK_EQ(mkdir(dir,0700),0);
  path(dir, "%s/esc/Z", build); CHECK_EQ(mkdir(dir,0700),0);
  path(dir, "%s/known-apps", backups); CHECK_EQ(mkdir(dir,0700),0);
  path(a, "%s/esc/A/motor_A_drive_offset.elf", build); tiny_elf32(a,true);
  path(z, "%s/esc/Z/motor_Z_drive_offset.elf", build); tiny_elf32(z,true);
  path(known, "%s/known-apps/old.elf", backups); tiny_elf32(known,true);
  uint8_t *flash = calloc(1,FLASHIMG_FLASH_SIZE); CHECK(flash != NULL);
  memcpy(flash+0x4020, "\x20\x21\x22\x23",4);
  CHECK_EQ(flashimg_installed(build, backups, flash, result, sizeof result),1); CHECK(!strcmp(result,a));
  CHECK_EQ(flashimg_installed(build, backups, flash, result, 2),-1);
  CHECK_EQ(unlink(a),0);
  CHECK_EQ(flashimg_installed(build, backups, flash, result, sizeof result),1); CHECK(!strcmp(result,z));
  CHECK_EQ(unlink(z),0);
  CHECK_EQ(flashimg_installed(build, backups, flash, result, sizeof result),1); CHECK(!strcmp(result,known));
  flash[0x4020] ^= 1;
  CHECK_EQ(flashimg_installed(build, backups, flash, result, sizeof result),0);
  free(flash);
}

static void preset(unsigned raw, unsigned scale, unsigned state, bool running)
{
  char memory[1024];
  path(memory, "%x=0,%x=0,%x=%x,%x=%x,%x=0,%x=0,%x=%x,%x=%x,%x=%x,%x=1",
       SENSOR, SENSOR+1, SENSOR+2, scale&255, SENSOR+3, scale>>8, SENSOR+4, SENSOR+5,
       SENSOR+6, raw&255, SENSOR+7, raw>>8, MCI+1, state, TICK);
  CHECK_EQ(setenv("FAKE_OCD_MEM",memory,1),0);
  if (running) CHECK_EQ(setenv("FAKE_OCD_READ_PAIR","20001020=1/2",1),0);
  else CHECK_EQ(unsetenv("FAKE_OCD_READ_PAIR"),0);
}

static void battery_case(const char *elf, int wheel, unsigned raw, unsigned scale, unsigned state,
                         bool running, int expected, const char *reason)
{
  preset(raw,scale,state,running);
  ocd_t *o = ocd_open(running ? SCRIPTED_OCD : FAKE_OCD, FLASHIMG_ESC_SN[wheel], "stm32g431", err,sizeof err);
  if (!o) fprintf(stderr, "%s\n",err);
  CHECK(o != NULL);
  flashimg_battery_t b;
  CHECK_EQ(flashimg_check_battery_off(o,elf,&b,err,sizeof err),expected);
  if (reason) CHECK(strstr(err,reason) != NULL);
  else { CHECK_NEAR(b.vbus_v, raw*scale/65536.0,1e-9); CHECK_EQ(b.mc_state,state); }
  ocd_close(o);
}

static void test_check_battery_off(void)
{
  char elf[1024]; path(elf,"%s/battery.elf",root); tiny_elf32(elf,true);
  flashimg_battery_symbols_t syms;
  CHECK_EQ(flashimg_battery_symbols(elf,&syms,err,sizeof err),0);
  CHECK_EQ(syms.sensor,SENSOR); CHECK_EQ(syms.mci,MCI); CHECK_EQ(syms.tick,TICK);
  for (int w = 0; w < 4; w++) {
    battery_case(elf,w,32768,24,0,true,-1,"Vbus 12.0 V");
    battery_case(elf,w,0,24,0,true,0,NULL);
    battery_case(elf,w,0,24,0,false,-1,"stale");
    battery_case(elf,w,0,24,6,true,-1,"not idle");
    battery_case(elf,w,32768,16,0,true,-1,"Vbus 8.0 V");
    battery_case(elf,w,32767,16,10,true,0,NULL);
    battery_case(elf,w,0,24,11,true,0,NULL);
  }
  flashimg_battery_t b = {.tick=0,.vbus_v=0,.mc_state=0};
  CHECK_EQ(flashimg_battery_validate(UINT32_MAX,&b,err,sizeof err),0);
  tiny_elf32(elf,false); CHECK_EQ(flashimg_battery_symbols(elf,&syms,err,sizeof err),-1);
  CHECK(strstr(err,"BusVoltageSensor") != NULL);
}

static void test_battery_guard(void)
{
  flashimg_battery_t b = {.tick=2,.vbus_v=0,.mc_state=0};
  CHECK_EQ(flashimg_battery_validate(1,&b,err,sizeof err),0);
  CHECK_EQ(flashimg_battery_validate(2,&b,err,sizeof err),-1); CHECK(strstr(err,"stale") != NULL);
  b.vbus_v=12; CHECK_EQ(flashimg_battery_validate(1,&b,err,sizeof err),-1);
  CHECK(strstr(err,"battery is ON") != NULL);
  b.vbus_v=8; CHECK_EQ(flashimg_battery_validate(1,&b,err,sizeof err),-1);
  b.vbus_v=7.999; CHECK_EQ(flashimg_battery_validate(1,&b,err,sizeof err),0);
  b.vbus_v=0;
  for (unsigned state=0; state<=255; state++) {
    b.mc_state=(uint8_t)state;
    CHECK_EQ(flashimg_battery_validate(1,&b,err,sizeof err),state==0 || state==10 || state==11 ? 0 : -1);
  }
  b.tick=0; b.mc_state=0; CHECK_EQ(flashimg_battery_validate(UINT32_MAX,&b,err,sizeof err),0);
}

static int parse(int n, const char *const args[], flashimg_options_t *o)
{
  return flashimg_parse(n,(char **)args,o,err,sizeof err);
}

static void test_argument_validation(void)
{
  flashimg_options_t o;
  for (int w = 0; w < 4; w++) {
    const char *a[] = {"esc_update",IK_WHEEL_CORNER[w],"--can"};
    CHECK_EQ(parse(3,a,&o),0); CHECK_EQ(o.wheel,w); CHECK(o.can && !o.battery_off);
    CHECK_EQ(IK_WHEEL_SELECT[o.wheel],1U<<w);
    CHECK_EQ(strlen(FLASHIMG_ESC_SN[w]),24);
    a[2]="--battery-off"; CHECK_EQ(parse(3,a,&o),0); CHECK(o.battery_off && !o.can);
  }
  const char *none[] = {"esc_update","NE"}; CHECK_EQ(parse(2,none,&o),-1);
  const char *both[] = {"esc_update","NE","--can","--battery-off"}; CHECK_EQ(parse(4,both,&o),-1);
  const char *boot[] = {"esc_update","NE","--battery-off","--target-in-boot"};
  CHECK_EQ(parse(4,boot,&o),-1); CHECK(strstr(err,"needs --can") != NULL);
  boot[2]="--can"; CHECK_EQ(parse(4,boot,&o),0); CHECK(o.target_in_boot);
  const char *keep[] = {"esc_update","SW","--battery-off","--keep-updater"};
  CHECK_EQ(parse(4,keep,&o),0); CHECK(o.keep_updater);
  keep[2]="--can"; CHECK_EQ(parse(4,keep,&o),0); /* Python accepts it, CAN always restores drive. */
  const char *paths[] = {"esc_update","--build","build x","NW","--can","--backups","backup x","--cli","cli x"};
  CHECK_EQ(parse(9,paths,&o),0); CHECK(!strcmp(o.build,"build x")); CHECK(!strcmp(o.backups,"backup x"));
  CHECK(!strcmp(o.cli,"cli x"));
  const char *eqpaths[] = {"esc_update","NE","--can","--build=build x","--backups=backup x","--cli=cli x"};
  CHECK_EQ(parse(6,eqpaths,&o),0); CHECK(!strcmp(o.build,"build x"));
  CHECK(!strcmp(o.backups,"backup x")); CHECK(!strcmp(o.cli,"cli x"));
  const char *bad[] = {"esc_update","NE","--can","--build"}; CHECK_EQ(parse(4,bad,&o),-1);
  bad[3]="--unknown"; CHECK_EQ(parse(4,bad,&o),-1);
  bad[3]="NW"; CHECK_EQ(parse(4,bad,&o),-1);
  bad[1]="FR"; CHECK_EQ(parse(3,bad,&o),-1);
  const char *help[] = {"esc_update","--help"}; CHECK_EQ(parse(2,help,&o),1);
  const char *no_wheel[] = {"esc_update","--can"}; CHECK_EQ(parse(2,no_wheel,&o),-1);
}

static void test_program_nucleo(void)
{
  char cli[1024], log[1024], args[1024];
  path(cli,"%s/STM32_Programmer_CLI",root); path(log,"%s/program.log",root); path(args,"%s/cli.args",root);
  const char script[] = "#!/bin/sh\nprintf '%s\\n' \"$@\" > \"$UPD_TEST_ARGS\"\n"
                        "printf '%b' \"$UPD_TEST_STDOUT\"\nprintf '%b' \"$UPD_TEST_STDERR\" >&2\n"
                        "exit \"$UPD_TEST_EXIT\"\n";
  put(cli,script,strlen(script)); CHECK_EQ(chmod(cli,0700),0);
  CHECK_EQ(setenv("UPD_TEST_ARGS",args,1),0); CHECK_EQ(setenv("UPD_TEST_EXIT","0",1),0);
  CHECK_EQ(setenv("UPD_TEST_STDERR","",1),0);
  CHECK_EQ(setenv("UPD_TEST_STDOUT","Download verified successfully\n",1),0);
  const char *elf = "app with spaces;$(false).elf";
  CHECK_EQ(flashimg_program_nucleo(cli,elf,log,err,sizeof err),0);
  char *text = get(args);
  CHECK(!strcmp(text,"-c\nport=SWD\nsn=" FLASHIMG_NUCLEO_SN "\n-w\napp with spaces;$(false).elf\n-v\n-rst\n")); free(text);
  text=get(log); CHECK(!strcmp(text,"Download verified successfully\n")); free(text);
  CHECK_EQ(setenv("UPD_TEST_STDOUT","Download complete\n",1),0);
  CHECK_EQ(flashimg_program_nucleo(cli,elf,log,err,sizeof err),-1);
  CHECK(strstr(err,"Nucleo programming failed") != NULL);
  CHECK_EQ(setenv("UPD_TEST_STDERR","Download verified successfully",1),0);
  CHECK_EQ(flashimg_program_nucleo(cli,elf,log,err,sizeof err),0);
  CHECK_EQ(setenv("UPD_TEST_EXIT","9",1),0);
  CHECK_EQ(flashimg_program_nucleo(cli,elf,log,err,sizeof err),-1);
  CHECK_EQ(setenv("UPD_TEST_EXIT","0",1),0);
  CHECK_EQ(setenv("UPD_TEST_STDOUT","Download \\033[32mverified\\033[0m successfully",1),0);
  CHECK_EQ(setenv("UPD_TEST_STDERR","",1),0);
  CHECK_EQ(flashimg_program_nucleo(cli,elf,log,err,sizeof err),0);
  CHECK_EQ(flashimg_program_nucleo("/nonexistent/STM32_Programmer_CLI",elf,log,err,sizeof err),-1);
  CHECK_EQ(flashimg_program_nucleo(cli,elf,"/nonexistent/log",err,sizeof err),-1);
}

static void test_bus_snapshot(void)
{
  ik_row_t before[4]={0}, after[4]={0};
  for (int i = 0; i < 4; i++) {
    before[i]=(ik_row_t){.valid=true,.status_count=10}; after[i]=before[i];
  }
  flashimg_live_rows(before,after);
  for (int i = 0; i < 4; i++) CHECK(!after[i].valid);
  for (int i = 0; i < 4; i++) { after[i]=before[i]; after[i].status_count++; }
  flashimg_live_rows(before,after);
  for (int i = 0; i < 4; i++) CHECK(after[i].valid);
  before[0].valid=false; after[0].status_count=0; after[1].valid=false;
  flashimg_live_rows(before,after); CHECK(!after[0].valid); CHECK(!after[1].valid);
  before[2].status_count=UINT32_MAX; after[2].status_count=1;
  flashimg_live_rows(before,after); CHECK(after[2].valid);
}

static void test_check_bus(void)
{
  ik_row_t rows[4]; double vbus[4];
  for (int i = 0; i < 4; i++) rows[i]=(ik_row_t){.valid=true,.state=0,.vbus_v=12.345,.diag_count=1,.mc_state="IDLE"};
  CHECK_EQ(flashimg_check_bus(rows,-1,vbus,err,sizeof err),0);
  for (int i = 0; i < 4; i++) CHECK_NEAR(vbus[i],12.35,1e-9);
  for (int w = 0; w < 4; w++) {
    ik_row_t good=rows[w]; rows[w].valid=false;
    CHECK_EQ(flashimg_check_bus(rows,-1,vbus,err,sizeof err),-1);
    CHECK(strstr(err,IK_WHEEL_POS[w]) && strstr(err,"no reply"));
    CHECK_EQ(flashimg_check_bus(rows,w,vbus,err,sizeof err),0); CHECK(isnan(vbus[w])); rows[w]=good;
    rows[w].state=10; rows[w].mc_state="FAULT_NOW";
    CHECK_EQ(flashimg_check_bus(rows,-1,vbus,err,sizeof err),-1); rows[w]=good;
    rows[w].active=true; CHECK_EQ(flashimg_check_bus(rows,-1,vbus,err,sizeof err),-1); rows[w]=good;
    rows[w].aligning=true; CHECK_EQ(flashimg_check_bus(rows,-1,vbus,err,sizeof err),-1); rows[w]=good;
    rows[w].diag_count=0; CHECK_EQ(flashimg_check_bus(rows,-1,vbus,err,sizeof err),-1); rows[w]=good;
    rows[w].vbus_v=NAN; CHECK_EQ(flashimg_check_bus(rows,-1,vbus,err,sizeof err),-1); rows[w]=good;
    rows[w].vbus_v=0; CHECK_EQ(flashimg_check_bus(rows,-1,vbus,err,sizeof err),0); rows[w]=good;
  }
  for (int i = 0; i < 4; i++) rows[i].valid=false;
  CHECK_EQ(flashimg_check_bus(rows,-1,vbus,err,sizeof err),-1);
  for (int i = 0; i < 4; i++) CHECK(strstr(err,IK_WHEEL_POS[i]) != NULL);
}

static void test_verify_flash(void)
{
  uint8_t *before = calloc(1,FLASHIMG_FLASH_SIZE), *after = calloc(1,FLASHIMG_FLASH_SIZE);
  CHECK(before && after);
  const uint8_t app[]={1,2,3,4,5}; uint32_t crc=crc32_ieee(app,sizeof app);
  memcpy(after+FLASHIMG_APP_OFFSET,app,sizeof app);
  after[FLASHIMG_META_OFFSET]=sizeof app;
  for (int i=0; i<4; i++) after[FLASHIMG_META_OFFSET+28+i]=(uint8_t)(crc>>(8*i));
  bool checks[3]; flashimg_verify_flash(before,after,app,sizeof app,crc,checks);
  CHECK(checks[0] && checks[1] && checks[2]);
  after[0]=1; flashimg_verify_flash(before,after,app,sizeof app,crc,checks); CHECK(!checks[0]); after[0]=0;
  after[FLASHIMG_APP_OFFSET+1]^=1;
  flashimg_verify_flash(before,after,app,sizeof app,crc,checks); CHECK(!checks[1]);
  after[FLASHIMG_META_OFFSET+28]^=1;
  flashimg_verify_flash(before,after,app,sizeof app,crc,checks); CHECK(!checks[2]);
  flashimg_verify_flash(before,after,app,FLASHIMG_FLASH_SIZE,crc,checks); CHECK(!checks[1]);
  free(before); free(after);
}

static void copy(const char *from, const char *to)
{
  uint8_t *b; size_t n; CHECK_EQ(flashimg_read_file(from,&b,&n),0); put(to,b,n); free(b);
}

static void session_dir(const char *backups, const char *prefix, char out[1024])
{
  DIR *d = opendir(backups); CHECK(d != NULL);
  struct dirent *e; bool found=false;
  while ((e=readdir(d))) if (!strncmp(e->d_name,prefix,strlen(prefix))) {
    CHECK(!found); path(out,"%s/%s",backups,e->d_name); found=true;
  }
  CHECK_EQ(closedir(d),0); CHECK(found);
}

static void test_backup_before_refusal(void)
{
  char build[1024], dir[1024], elf[1024], bin[1024], backups[1024], args[1024], cli[1024], rpc[1024];
  path(build,"%s/refusal-build",root); CHECK_EQ(mkdir(build,0700),0);
  path(dir,"%s/esc",build); CHECK_EQ(mkdir(dir,0700),0);
  path(dir,"%s/updater",build); CHECK_EQ(mkdir(dir,0700),0);
  path(cli,"%s/STM32_Programmer_CLI",root);
  path(args,"%s/must-not-program",root); CHECK_EQ(setenv("UPD_TEST_ARGS",args,1),0);
  path(rpc,"%s/refusal-rpc.log",root); CHECK_EQ(setenv("FAKE_OCD_READ_LOG",rpc,1),0);
  CHECK_EQ(setenv("OPENOCD",SCRIPTED_OCD,1),0);
  for (int w=0; w<4; w++) {
    path(dir,"%s/esc/%s",build,IK_WHEEL_CORNER[w]); CHECK_EQ(mkdir(dir,0700),0);
    path(elf,"%s/motor_%s_drive_offset.elf",dir,IK_WHEEL_CORNER[w]); tiny_elf32(elf,true);
    path(bin,"%s/motor_%s_drive_offset.bin",dir,IK_WHEEL_CORNER[w]); put(bin,"abcd",4);
    path(dir,"%s/updater/%s",build,IK_WHEEL_CORNER[w]); CHECK_EQ(mkdir(dir,0700),0);
    path(elf,"%s/sfurs_firmware_v2_CM7.elf",dir); copy("/proc/self/exe",elf);
    path(backups,"%s/refusal-backups-%s",root,IK_WHEEL_CORNER[w]);
    preset(32768,24,0,true);
    flashimg_options_t o={.wheel=w,.battery_off=true,.build=build,.backups=backups,.cli=cli};
    CHECK_EQ(flashimg_install(&o,err,sizeof err),-1);
    if (!strstr(err,"Vbus 12.0 V")) fprintf(stderr,"unexpected installer refusal: %s\n",err);
    CHECK(strstr(err,"Vbus 12.0 V") != NULL);
    CHECK(access(args,F_OK) != 0);
    char session[1024], file[1024]; session_dir(backups,"update-",session);
    path(file,"%s/before.bin",session); uint8_t *flash; size_t n;
    CHECK_EQ(flashimg_read_file(file,&flash,&n),0); CHECK_EQ(n,FLASHIMG_FLASH_SIZE);
    for (size_t i=0; i<n; i++) CHECK_EQ(flash[i],i&255);
    char hash[65]; sha256_hex(flash,n,hash); free(flash);
    path(file,"%s/session.json",session); char *text=get(file);
    const char *keys[]={"wheel","selector","esc_probe","app_elf","app_sha256","app_crc32","app_bytes",
                        "battery_off_confirmed","complete","before_sha256"};
    for (size_t i=0; i<sizeof keys/sizeof keys[0]; i++) CHECK(strstr(text,keys[i]) != NULL);
    CHECK(strstr(text,FLASHIMG_ESC_SN[w]) != NULL); CHECK(strstr(text,hash) != NULL);
    CHECK(strstr(text,"\"complete\": false") != NULL); free(text);
  }
  char *text=get(rpc); CHECK(strstr(text,"mww ") == NULL); free(text);
  CHECK_EQ(unsetenv("FAKE_OCD_READ_LOG"),0); CHECK_EQ(unsetenv("OPENOCD"),0);
}

int main(int argc, char **argv)
{
  strcpy(root,"/tmp/test-esc-update-XXXXXX"); CHECK(mkdtemp(root) != NULL);
  const struct { const char *name; void (*run)(void); } cases[] = {
    {"flash_matches_sections",test_flash_matches_sections},
    {"flash_matches_elf64",test_flash_matches_elf64},
    {"installed_app_elf",test_installed_app_elf},
    {"check_battery_off",test_check_battery_off},
    {"battery_guard",test_battery_guard},
    {"argument_validation",test_argument_validation},
    {"program_nucleo",test_program_nucleo},
    {"bus_snapshot",test_bus_snapshot},
    {"check_bus",test_check_bus},
    {"verify_flash",test_verify_flash},
    {"backup_before_refusal",test_backup_before_refusal}
  };
  unsigned ran=0;
  for (size_t i=0; i<sizeof cases/sizeof cases[0]; i++) {
    if (argc > 1 && strcmp(argv[1],cases[i].name)) continue;
    cases[i].run(); ran++;
  }
  CHECK(ran > 0);
  printf("%u ESC updater cases passed; fixtures: %s\n",ran,root);
  return 0;
}
