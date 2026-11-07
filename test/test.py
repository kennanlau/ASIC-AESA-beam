# Tests for the beam steering engine (tt_um_kennanlau_beam_steer).
# Bit-exact Python reference + physics check against the ideal phase-shift formula.
# Signals are sampled on falling clock edges so the same tests work for RTL and gate-level runs.
import hashlib, math, os, pathlib, random
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, ClockCycles

GOLD = pathlib.Path(__file__).parent / "golden" / "angle_table.sha256"
SEL_DXL, SEL_DXH, SEL_DYL, SEL_DYH, SEL_AX, SEL_AY, SEL_CLR = range(7)


def lut(a):
    return round(1024 * math.sin(math.radians(a)))


def d_from_angle(deg):
    """Signed angle in degrees -> 12-bit phase increment (1/16 steps), clamped to +/-90."""
    v = lut(min(abs(deg), 90))
    return -v if deg < 0 else v


def ref_phase(dx, dy, n):
    """Bit-exact model of the RTL: floor((8 - (ix*Dx + iy*Dy)) / 16) mod 128."""
    ix, iy = n & 3, n >> 2
    return ((8 - (ix * dx + iy * dy)) >> 4) & 0x7F


def ideal_steps(tx, ty, n):
    """Physics: phase = -pi*(ix*sin(tx) + iy*sin(ty)) at half-wavelength spacing, in 2*pi/128 steps."""
    ix, iy = n & 3, n >> 2
    return (-64 * (ix * math.sin(math.radians(tx)) + iy * math.sin(math.radians(ty)))) % 128


def check_golden(path, text):
    h = hashlib.sha256(text.encode()).hexdigest()
    if os.environ.get("GOLDEN_UPDATE") == "1":
        path.write_text(h + "\n")
    assert path.exists(), f"missing golden {path.name}: rerun with GOLDEN_UPDATE=1 only after review"
    assert path.read_text().strip() == h, f"golden mismatch for {path.name}"


class Pins:
    def __init__(self, dut): self.dut, self.ui = dut, 0
    def apply(self): self.dut.ui_in.value = self.ui


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
    pins = Pins(dut); dut.ena.value = 1; dut.uio_in.value = 0; pins.apply()
    dut.rst_n.value = 0; await ClockCycles(dut.clk, 5); dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)
    return pins


async def wr(dut, pins, sel, val):
    await FallingEdge(dut.clk)
    pins.ui = (pins.ui & 0xF0) | (sel << 1); pins.apply(); dut.uio_in.value = val & 0xFF
    await ClockCycles(dut.clk, 2); await FallingEdge(dut.clk)
    pins.ui |= 1; pins.apply(); await ClockCycles(dut.clk, 5); await FallingEdge(dut.clk)
    pins.ui &= ~1; pins.apply(); await ClockCycles(dut.clk, 4)


async def read(dut, pins, n):
    await FallingEdge(dut.clk)
    pins.ui = (pins.ui & 0x0F) | (n << 4); pins.apply()
    await ClockCycles(dut.clk, 5); await FallingEdge(dut.clk)
    return int(dut.uo_out.value) & 0x7F


async def read_all(dut, pins):
    return [await read(dut, pins, n) for n in range(16)]


async def set_direct(dut, pins, dx, dy):
    await wr(dut, pins, SEL_DXL, dx & 0xFF); await wr(dut, pins, SEL_DXH, (dx >> 8) & 0xF)
    await wr(dut, pins, SEL_DYL, dy & 0xFF); await wr(dut, pins, SEL_DYH, (dy >> 8) & 0xF)


def circ(a, b):
    d = (a - b) % 128
    return min(d, 128 - d)


@cocotb.test()
async def reset_state_all_zero(dut):
    pins = await setup(dut)
    assert await read_all(dut, pins) == [0] * 16


@cocotb.test()
async def angle_x_all_degrees_bit_exact_and_golden(dut):
    pins = await setup(dut)
    rows, worst = [], 0.0
    for theta in range(-90, 91):
        await wr(dut, pins, SEL_AX, theta)
        got = await read_all(dut, pins)
        exp = [ref_phase(d_from_angle(theta), 0, n) for n in range(16)]
        assert got == exp, (theta, got, exp)
        worst = max(worst, max(circ(got[n], ideal_steps(theta, 0, n)) for n in range(16)))
        rows.append(f"{theta}:" + ",".join(map(str, got)))
    dut._log.info(f"worst error vs ideal phase over -90..90 deg: {worst:.3f} steps ({worst*2.8125:.2f} deg)")
    assert worst <= 1.0
    check_golden(GOLD, "\n".join(rows))


@cocotb.test()
async def angle_2d_matches_physics(dut):
    pins = await setup(dut)
    for tx in range(-60, 61, 15):
        for ty in range(-60, 61, 15):
            await wr(dut, pins, SEL_AX, tx); await wr(dut, pins, SEL_AY, ty)
            got = await read_all(dut, pins)
            exp = [ref_phase(d_from_angle(tx), d_from_angle(ty), n) for n in range(16)]
            assert got == exp, (tx, ty)
            assert max(circ(got[n], ideal_steps(tx, ty, n)) for n in range(16)) <= 1.0, (tx, ty)


@cocotb.test()
async def direct_mode_random(dut):
    pins = await setup(dut)
    rng = random.Random(1234)
    for _ in range(40):
        dx, dy = rng.randint(-2048, 2047), rng.randint(-2048, 2047)
        await set_direct(dut, pins, dx, dy)
        assert await read_all(dut, pins) == [ref_phase(dx, dy, n) for n in range(16)], (dx, dy)


@cocotb.test()
async def clamp_and_clear(dut):
    pins = await setup(dut)
    await wr(dut, pins, SEL_AX, 90); p90 = await read_all(dut, pins)
    await wr(dut, pins, SEL_AX, 120); assert await read_all(dut, pins) == p90      # clamped to 90
    await wr(dut, pins, SEL_AX, -90); m90 = await read_all(dut, pins)
    await wr(dut, pins, SEL_AX, -128); assert await read_all(dut, pins) == m90     # clamped to -90
    await wr(dut, pins, SEL_AY, 45); await wr(dut, pins, SEL_CLR, 0)
    assert await read_all(dut, pins) == [0] * 16


@cocotb.test()
async def write_ack_and_held_strobe(dut):
    pins = await setup(dut)
    ack0 = int(dut.uo_out.value) >> 7
    await FallingEdge(dut.clk)
    pins.ui = (SEL_AX << 1) | 1; pins.apply(); dut.uio_in.value = 30
    await ClockCycles(dut.clk, 200)                                    # strobe held high
    await FallingEdge(dut.clk)
    assert int(dut.uo_out.value) >> 7 == ack0 ^ 1                       # exactly one write
    pins.ui = 0; pins.apply(); await ClockCycles(dut.clk, 5)
    await wr(dut, pins, SEL_AY, 0)
    await FallingEdge(dut.clk)
    assert int(dut.uo_out.value) >> 7 == ack0                           # toggled back
