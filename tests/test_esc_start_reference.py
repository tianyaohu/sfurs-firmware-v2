#!/usr/bin/env python3
"""Regression (2026-10-17 run R1): at motor start MCSDK calls HALL_Clear() and
then STC_ForceSpeedReferenceToCurrentSpeed(), which starts the speed ramp at
SPD_GetAvrgMecSpeedUnit(). That average is only recomputed while running, so
HALL_Clear() must zero it, or a wheel pushed while IDLE starts from a stale
speed (FR started at -92 RPM and limit-cycled). Source-level check: the Hall
driver needs the STM32 timer peripheral to run on the host."""
import pathlib
import re
import unittest

MC = pathlib.Path(__file__).resolve().parents[1] / "HW/B-G431B-ESC1/motor_config/Src"


def body(text, signature):
    start = text.index(signature)
    depth, i = 0, text.index("{", start)
    for j in range(i, len(text)):
        depth += {"{": 1, "}": -1}.get(text[j], 0)
        if depth == 0:
            return text[i:j]
    raise ValueError(signature)


class StartReference(unittest.TestCase):
    def test_hall_clear_zeroes_the_average_speed(self):
        clear = body((MC / "hall_speed_pos_fdbk.c").read_text(), "void HALL_Clear(")
        for field in ("_Super.hAvrMecSpeedUnit", "_Super.hElSpeedDpp", "AvrElSpeedDpp"):
            self.assertRegex(clear, rf"pHandle->{re.escape(field)} = 0;", field)

    def test_clear_runs_before_the_reference_is_forced(self):
        tasks = (MC / "mc_tasks_foc.c").read_text()
        clear = tasks.index("HALL_Clear(&HALL_M1);")
        force = tasks.index("STC_ForceSpeedReferenceToCurrentSpeed(")
        self.assertLess(clear, force)
        self.assertNotIn("SPD_GetAvrgMecSpeedUnit", tasks[clear:force])


if __name__ == "__main__":
    unittest.main()
