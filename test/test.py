# SPDX-License-Identifier: Apache-2.0
# Tests for the CRC-protected UART command interface.
# The testbench plays the role of the host PC: it drives ui_in[0] (chip RX)
# and watches uo_out[0] (chip TX).

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

# clocks per UART bit for each baud_sel value (50 MHz clock)
BIT_CLKS = {0: 434, 1: 868, 2: 217, 3: 16}

START_BYTE = 0xA5
ACK, NAK = 0x06, 0x15
CMD_WRITE, CMD_READ, CMD_PING = 0x01, 0x02, 0x03
GPIO_OUT, GPIO_DIR, PWM_DUTY, STATUS = 0, 1, 2, 3


def crc8(data, poly=0x07):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def bit(sig, n):
    return (int(sig.value) >> n) & 1


class Host:
    """Fake UART host connected to the chip pins."""

    def __init__(self, dut, baud_sel):
        self.dut = dut
        self.baud_sel = baud_sel
        self.bit_clks = BIT_CLKS[baud_sel]
        self.rx_bytes = []  # bytes received from the chip
        self.task = None

    def set_line(self, level):
        self.dut.ui_in.value = (self.baud_sel << 1) | level

    async def send_byte(self, byte):
        bits = [0] + [(byte >> i) & 1 for i in range(8)] + [1]
        for b in bits:
            self.set_line(b)
            await ClockCycles(self.dut.clk, self.bit_clks)

    async def send_packet(self, cmd, addr, data, corrupt=False):
        crc = crc8([cmd, addr, data])
        if corrupt:
            crc ^= 0x01
        for b in (START_BYTE, cmd, addr, data, crc):
            await self.send_byte(b)

    async def monitor(self):
        """Background UART receiver watching the chip's TX pin."""
        step = max(1, self.bit_clks // 8)
        while True:
            while bit(self.dut.uo_out, 0) == 1:
                await ClockCycles(self.dut.clk, step)
            await ClockCycles(self.dut.clk, self.bit_clks // 2)
            if bit(self.dut.uo_out, 0) == 1:
                continue  # glitch, not a start bit
            value = 0
            for i in range(8):
                await ClockCycles(self.dut.clk, self.bit_clks)
                value |= bit(self.dut.uo_out, 0) << i
            await ClockCycles(self.dut.clk, self.bit_clks)  # stop bit
            self.rx_bytes.append(value)

    async def wait_bytes(self, n, max_bits=80):
        limit = max_bits * self.bit_clks
        waited = 0
        while len(self.rx_bytes) < n:
            await ClockCycles(self.dut.clk, 4)
            waited += 4
            assert waited < limit, f"timeout: got {self.rx_bytes}, expected {n} byte(s)"

    async def transact(self, cmd, addr, data, corrupt=False, n_reply=1):
        """Send one packet, return every byte the chip sent back."""
        self.rx_bytes.clear()
        await self.send_packet(cmd, addr, data, corrupt)
        await self.wait_bytes(n_reply)
        # wait a bit: catches unexpected extra bytes and lets the chip go idle
        await ClockCycles(self.dut.clk, 4 * self.bit_clks)
        return list(self.rx_bytes)


async def reset(dut, baud_sel=3):
    dut.ena.value = 1
    dut.ui_in.value = (baud_sel << 1) | 1  # rx line idles high
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)
    host = Host(dut, baud_sel)
    host.task = cocotb.start_soon(host.monitor())
    return host


async def start(dut, baud_sel=3):
    dut._log.info("Start")
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())  # 50 MHz
    dut._log.info("Reset")
    return await reset(dut, baud_sel)


async def pwm_high_count(dut, n=256):
    high = 0
    for _ in range(n):
        await ClockCycles(dut.clk, 1)
        high += bit(dut.uo_out, 3)
    return high


@cocotb.test()
async def test_ping(dut):
    host = await start(dut)
    assert await host.transact(CMD_PING, 0, 0) == [ACK]
    assert bit(dut.uo_out, 1) == 1, "pkt_ok should be high"
    assert bit(dut.uo_out, 2) == 0, "pkt_err should be low"


@cocotb.test()
async def test_registers(dut):
    host = await start(dut)
    assert await host.transact(CMD_WRITE, GPIO_DIR, 0x0F) == [ACK]
    assert await host.transact(CMD_WRITE, GPIO_OUT, 0x0A) == [ACK]
    assert int(dut.uio_oe.value) == 0x0F
    assert int(dut.uio_out.value) == 0x0A
    assert await host.transact(CMD_READ, GPIO_OUT, 0, n_reply=2) == [ACK, 0x0A]
    assert await host.transact(CMD_READ, GPIO_DIR, 0, n_reply=2) == [ACK, 0x0F]
    assert await host.transact(CMD_WRITE, PWM_DUTY, 0x40) == [ACK]
    assert await host.transact(CMD_READ, PWM_DUTY, 0, n_reply=2) == [ACK, 0x40]


@cocotb.test()
async def test_pwm(dut):
    host = await start(dut)
    assert await host.transact(CMD_WRITE, PWM_DUTY, 0x80) == [ACK]
    assert await pwm_high_count(dut) == 128
    assert await host.transact(CMD_WRITE, PWM_DUTY, 0x00) == [ACK]
    assert await pwm_high_count(dut) == 0
    assert await host.transact(CMD_WRITE, PWM_DUTY, 0xFF) == [ACK]
    assert await pwm_high_count(dut) == 255


@cocotb.test()
async def test_bad_crc(dut):
    host = await start(dut)
    assert await host.transact(CMD_WRITE, GPIO_DIR, 0x0F) == [ACK]
    assert await host.transact(CMD_WRITE, GPIO_OUT, 0x05, corrupt=True) == [NAK]
    assert bit(dut.uo_out, 2) == 1, "pkt_err should be high"
    assert bit(dut.uo_out, 1) == 0, "pkt_ok should be low"
    assert int(dut.uio_out.value) == 0x00, "bad packet must not write the register"
    reply = await host.transact(CMD_READ, STATUS, 0, n_reply=2)
    assert reply[0] == ACK and (reply[1] & 0x0F) == 1


@cocotb.test()
async def test_bad_command_and_address(dut):
    host = await start(dut)
    assert await host.transact(0x7F, 0, 0) == [NAK]                # unknown command
    assert await host.transact(CMD_WRITE, 0x09, 0x55) == [NAK]     # address out of range
    assert await host.transact(CMD_READ, 0x80, 0) == [NAK]         # address out of range
    reply = await host.transact(CMD_READ, STATUS, 0, n_reply=2)
    assert reply[0] == ACK and (reply[1] & 0x0F) == 3
    assert await host.transact(CMD_WRITE, STATUS, 0) == [ACK]      # clears the counter
    reply = await host.transact(CMD_READ, STATUS, 0, n_reply=2)
    assert reply[0] == ACK and (reply[1] & 0x0F) == 0


@cocotb.test()
async def test_junk_bytes_ignored(dut):
    host = await start(dut)
    junk = [b for b in range(256) if b != START_BYTE]
    for _ in range(20):
        await host.send_byte(random.choice(junk))
    await ClockCycles(dut.clk, 4 * host.bit_clks)
    assert host.rx_bytes == [], "junk must not produce any reply"
    assert bit(dut.uo_out, 2) == 0
    assert await host.transact(CMD_PING, 0, 0) == [ACK]            # still in sync
    reply = await host.transact(CMD_READ, STATUS, 0, n_reply=2)
    assert (reply[1] & 0x0F) == 0


@cocotb.test()
async def test_timeout_on_cut_packet(dut):
    host = await start(dut)
    await host.send_byte(START_BYTE)
    await host.send_byte(CMD_WRITE)                                # packet cut short
    await ClockCycles(dut.clk, 70000)
    assert bit(dut.uo_out, 2) == 1, "timeout should raise pkt_err"
    assert host.rx_bytes == []
    assert await host.transact(CMD_PING, 0, 0) == [ACK]            # recovered
    reply = await host.transact(CMD_READ, STATUS, 0, n_reply=2)
    assert (reply[1] & 0x0F) == 1


@cocotb.test()
async def test_gpio_input_status(dut):
    host = await start(dut)
    dut.uio_in.value = 0x05
    reply = await host.transact(CMD_READ, STATUS, 0, n_reply=2)
    assert reply == [ACK, 0x50]


@cocotb.test()
async def test_all_baud_rates(dut):
    host = await start(dut, 0)
    for sel in (0, 1, 2, 3):
        if sel != 0:
            host.task.cancel()
            host = await reset(dut, sel)
        assert await host.transact(CMD_PING, 0, 0) == [ACK], f"baud_sel {sel}"
        assert await host.transact(CMD_WRITE, PWM_DUTY, 0x5A) == [ACK], f"baud_sel {sel}"
        assert await host.transact(CMD_READ, PWM_DUTY, 0, n_reply=2) == [ACK, 0x5A]
