//! TIMER0 driver and BLE interval tick math for the scheduler.
use super::pac;

/// Timer ticks per 0.625 ms BLE unit at 31.25 kHz.
pub const TICKS_PER_UNIT: u32 = 625;
/// Denominator for the fractional tick accumulator.
pub const TICKS_PER_UNIT_DENOM: u32 = 32;

/// Accumulates the fractional part of interval tick conversion so that
/// long schedules never drift.
pub struct IntervalAccum {
    whole: u32,
    frac_part: u32,
    frac: u32,
}

impl IntervalAccum {
    /// Create a fresh accumulator for 0.625 ms units.
    pub const fn new() -> Self {
        IntervalAccum {
            whole: TICKS_PER_UNIT / TICKS_PER_UNIT_DENOM,
            frac_part: TICKS_PER_UNIT % TICKS_PER_UNIT_DENOM,
            frac: 0,
        }
    }

    /// Create an accumulator for 1.25 ms connection interval units.
    pub const fn new_125ms() -> Self {
        IntervalAccum {
            whole: 39,
            frac_part: 2,
            frac: 0,
        }
    }

    /// Ticks to add for an interval of `units`.
    pub fn next(&mut self, units: u16) -> u32 {
        let base = u32::from(units) * self.whole;
        self.frac += u32::from(units) * self.frac_part;
        let carry = self.frac / TICKS_PER_UNIT_DENOM;
        self.frac %= TICKS_PER_UNIT_DENOM;
        base + carry
    }
}

/// Ticks for a scan window of `units`, rounded up.
pub const fn window_ticks(units: u16) -> u32 {
    (units as u32 * TICKS_PER_UNIT).div_ceil(TICKS_PER_UNIT_DENOM)
}

/// Ticks for a timeout of `micros` microseconds, rounded up.
pub const fn timeout_ticks(micros: u32) -> u32 {
    micros.div_ceil(32)
}

/// TIMER0 driver running at 31.25 kHz (32 us ticks, 24-bit).
pub struct BtTimer {
    regs: pac::TIMER0,
}

impl BtTimer {
    /// Wrap the TIMER0 peripheral.
    pub fn new(regs: pac::TIMER0) -> Self {
        BtTimer { regs }
    }

    /// Configure the timer (timer mode, 31.25 kHz, 24-bit) and start it.
    pub fn init(&self) {
        let r = &self.regs;
        r.mode.write(|w| w.mode().timer());
        r.prescaler.write(|w| unsafe { w.prescaler().bits(9) });
        r.bitmode.write(|w| w.bitmode()._24bit());
        r.tasks_clear.write(|w| unsafe { w.bits(1) });
        r.tasks_start.write(|w| unsafe { w.bits(1) });
    }

    /// Current timer value (captured via CC1).
    pub fn now(&self) -> u32 {
        let r = &self.regs;
        r.tasks_capture[1].write(|w| unsafe { w.bits(1) });
        r.cc[1].read().cc().bits()
    }

    /// Arm a compare on CC0.
    pub fn set_compare(&self, value: u32) {
        self.regs.cc[0].write(|w| unsafe { w.cc().bits(value) });
    }

    /// True when the CC0 compare event is pending.
    pub fn compare_pending(&self) -> bool {
        self.regs.events_compare[0].read().bits() != 0
    }

    /// Clear the CC0 compare event.
    pub fn clear_compare(&self) {
        self.regs.events_compare[0].write(|w| w);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interval_ticks_are_exact() {
        let mut acc = IntervalAccum::new();
        assert_eq!(acc.next(32), 625);
        assert_eq!(acc.next(32), 625);
    }

    #[test]
    fn interval_ticks_follow_fractional_accumulation() {
        let mut acc = IntervalAccum::new();
        let mut total = 0u64;
        for _ in 0..1000 {
            total += acc.next(33) as u64;
        }
        assert_eq!(total, (1000 * 33 * 625) / 32);
    }

    #[test]
    fn max_interval_fits_timer() {
        let mut acc = IntervalAccum::new();
        let ticks = acc.next(16384);
        assert_eq!(ticks, 320_000);
        assert!(ticks < (1 << 24));
    }

    #[test]
    fn window_ticks_round_up() {
        assert_eq!(window_ticks(4), 79);
        assert_eq!(window_ticks(32), 625);
    }

    #[test]
    fn timeout_ticks_round_up() {
        assert_eq!(timeout_ticks(150), 5);
        assert_eq!(timeout_ticks(160), 5);
        assert_eq!(timeout_ticks(0), 0);
    }
}
