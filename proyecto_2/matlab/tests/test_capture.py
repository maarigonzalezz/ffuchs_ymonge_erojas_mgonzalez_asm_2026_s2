"""Pruebas sin pyserial ni hardware: python3 -m unittest discover -s tests -p 'test_*.py'."""
import importlib.util
import io
import json
import math
from pathlib import Path
import random
import tempfile
import unittest

MODULE = Path(__file__).resolve().parents[1] / 'captura_serial.py'
spec = importlib.util.spec_from_file_location('capture_under_test', MODULE)
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)


class Clock:
    def __init__(self): self.value = 0
    def __call__(self): return self.value


class Serial:
    def __init__(self, lines, clock):
        self.lines = iter(lines)
        self.clock = clock
        self.sent = []
    def write(self, data): self.sent.append(data)
    def readline(self, size=513):
        self.clock.value += .01
        item = next(self.lines, b'')
        if isinstance(item, BaseException): raise item
        return item[:size]


def transcript(result='duracion_maxima'):
    return [b'#START_CSV\n', (capture.HEADER+'\n').encode(),
            b'-20,PRE,150,0,20,400,400,41.8879,1\n',
            b'0,BRAKE,0,0.4,60,400,400,41.8879,2\n',
            b'20,BRAKE,0,0.4,60,390,397,41.5737,3\n',
            b'#END_CSV\n', f'#RESULT {result}\n'.encode()]


class CaptureTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = Path(self.temp.name)/'run.csv'
        self.addCleanup(self.temp.cleanup)
    def run_capture(self, lines, command='step 150 .4 20 20 1000'):
        clock = Clock()
        serial = Serial(lines, clock)
        try:
            capture.capture(serial, self.path, command, timeout=.05, clock=clock)
        finally:
            self.assertEqual(serial.sent[-1], b'x\n')
        return serial
    def test_valid_capture_and_metadata(self):
        self.run_capture(transcript())
        self.assertEqual(capture.validate_csv(self.path), 3)
        self.assertFalse(self.path.with_suffix('.partial.csv').exists())
        meta = json.loads(self.path.with_suffix('.capture.json').read_text())
        self.assertEqual(meta['status'], 'complete')
    def test_cancelled_or_sensorless_never_published(self):
        for reason in ['cancelado: motor PWM=0, freno liberado', 'duracion_maxima_sin_velocidad_valida', 'error_buffer_serial: captura incompleta']:
            with self.subTest(reason=reason), tempfile.TemporaryDirectory() as folder:
                self.path = Path(folder)/'run.csv'
                with self.assertRaises(capture.CaptureError): self.run_capture(transcript(reason))
                self.assertFalse(self.path.exists())
                self.assertTrue(self.path.with_suffix('.partial.csv').exists())
                self.assertEqual(json.loads(self.path.with_suffix('.capture.json').read_text())['status'], 'incomplete')
    def test_missing_end_or_result(self):
        for lines in [transcript()[:5], transcript()[:6]]:
            with self.subTest(lines=len(lines)), tempfile.TemporaryDirectory() as folder:
                self.path=Path(folder)/'run.csv'
                with self.assertRaises(capture.CaptureError): self.run_capture(lines)
                self.assertFalse(self.path.exists())
    def test_rejected_before_start(self):
        with self.assertRaises(capture.CaptureError): self.run_capture([b'#ERR comando invalido\n'])
    def test_no_overwrite(self):
        self.path.write_text('original')
        with self.assertRaises(FileExistsError):
            capture.capture(Serial([],Clock()),self.path)
        self.assertEqual(self.path.read_text(), 'original')
    def test_binary_or_oversized_lines(self):
        for line in [b'\xff\n', b'bad\x00data\n', b'A'*513+b'\n']:
            with tempfile.TemporaryDirectory() as folder:
                self.path=Path(folder)/'run.csv'
                with self.assertRaises(capture.CaptureError): self.run_capture([line])
    def test_fragmented_lines(self):
        lines=[]
        for line in transcript(): lines.extend([line[:3],line[3:]])
        self.run_capture(lines)
    def test_duplicate_markers(self):
        for lines in [[b'#END_CSV\n'],[b'#START_CSV\n',b'#START_CSV\n']]:
            with tempfile.TemporaryDirectory() as folder:
                self.path=Path(folder)/'run.csv'
                with self.assertRaises(capture.CaptureError): self.run_capture(lines)
    def test_connection_failure_and_interrupt_stop(self):
        for error in [OSError('desconectado'), KeyboardInterrupt()]:
            with tempfile.TemporaryDirectory() as folder:
                self.path=Path(folder)/'run.csv'
                with self.assertRaises(type(error)): self.run_capture([error])
    def test_corrupt_csv_rows(self):
        for bad in [b'20,BRAKE,1,.4,60,390,397,41.5737,3\n',
                    b'0,BRAKE,0,.4,60,390,397,41.5737,3\n',
                    b'20,BRAKE,0,.4,60,nan,397,41.5737,3\n',
                    b'20,BRAKE,0,.4,60,390,397,999,3\n',
                    b'20,BRAKE,0,.4,60,390,397,41.5737,-1\n',
                    b'20,BRAKE,0,.7,60,390,397,41.5737,3\n']:
            with tempfile.TemporaryDirectory() as folder:
                self.path=Path(folder)/'run.csv';lines=transcript();lines[4]=bad
                with self.assertRaises(capture.CaptureError): self.run_capture(lines)
                self.assertFalse(self.path.exists())
    def test_command_mismatch(self):
        lines=transcript()
        lines[3]=lines[3].replace(b'0.4',b'0.7')
        lines[4]=lines[4].replace(b'0.4',b'0.7')
        with self.assertRaises(capture.CaptureError): self.run_capture(lines)
        self.assertFalse(self.path.exists())
    def test_timeout_scaling_and_invalid_commands(self):
        wait,data=capture.capture_timeouts('step 1 0 600000 600000 600000')
        self.assertGreater(wait,600);self.assertGreater(data,1200)
        for command in ['m 1','step 0 0 20 0 20','step 1 nan 20 0 20','step 1 0 19 0 20','step 1 0 20 0 600001']:
            with self.assertRaises(ValueError): capture.capture_timeouts(command)
        for timeout in [0,-1,math.inf,math.nan]:
            with self.assertRaises(ValueError): capture.capture_timeouts('',timeout)
    def test_fuzz_protocol_never_completes_without_result(self):
        rng=random.Random(1110)
        for _ in range(10000):
            protocol=capture.CaptureProtocol(io.StringIO())
            lines=[rng.choice(['#START_CSV','#END_CSV',capture.HEADER,'garbage','#ERR nope','#WARN nope']) for _ in range(rng.randrange(1,20))]
            try:
                for line in lines: protocol.accept(line)
            except capture.CaptureError: pass
            self.assertNotEqual(protocol.state,'DONE')

    def test_runpid_capture_and_validation(self):
        lines = [
            b'#START_CSV\n',
            (capture.HEADER_RUNPID + '\n').encode(),
            b'-20,PRE,PID,180,40.0,40.0,40.1,0,0,0,0,0,44,10,0\n',
            b'0,PID,PID,0,40.0,40.0,40.0,0.0,0.0,0.0,0.0,0.0,44,12,1\n',
            b'20,PID,PID,0,39.6,39.9,40.0,0.3,0.03,0.02,0.01,0.0,49.12,14,0\n',
            b'#END_CSV\n',
            b'#RESULT parada_detectada\n',
        ]
        self.run_capture(lines, command='runpid 180 20 20 1000')
        self.assertEqual(capture.validate_csv(self.path, 'runpid 180 20 20 1000'), 3)

    def test_runpid_rejects_bad_error_and_flags(self):
        bad_lines = [
            b'20,PID,PID,0,39.6,39.9,40.0,5.0,0.03,0.02,0.01,0.0,49.12,14,0\n',
            b'20,PID,PID,0,39.6,39.9,40.0,0.3,0.03,0.02,0.01,0.0,49.12,14,-1\n',
            b'20,PID,AUTO,0,39.6,39.9,40.0,0.3,0.03,0.02,0.01,0.0,49.12,14,0\n',
        ]
        for bad in bad_lines:
            with tempfile.TemporaryDirectory() as folder:
                self.path = Path(folder) / 'pid.csv'
                lines = [
                    b'#START_CSV\n',
                    (capture.HEADER_RUNPID + '\n').encode(),
                    b'-20,PRE,PID,180,40.0,40.0,40.1,0,0,0,0,0,44,10,0\n',
                    b'0,PID,PID,0,40.0,40.0,40.0,0.0,0.0,0.0,0.0,0.0,44,12,1\n',
                    bad,
                    b'#END_CSV\n',
                    b'#RESULT parada_detectada\n',
                ]
                with self.assertRaises(capture.CaptureError):
                    self.run_capture(lines, command='runpid 180 20 20 1000')
                self.assertFalse(self.path.exists())


def transcript_pid(result='w_final_alcanzada'):
    """Lazo cerrado coherente: u = clamp(P + I + D), error = w_meas - w_ref, rampa decreciente."""
    return [b'#START_CSV\n', (capture.HEADER_PID+'\n').encode(),
            b'-20.000,PRE,OPEN,200,0.0000,40.0000,40.0283,0.0000,0.0000,0.0000,0.0000,0.0000,44.00,650,0\n',
            b'0.000,BRAKE,PID,0,40.0000,40.0000,40.0283,0.0000,0.0000,0.0000,0.0000,0.0000,44.00,651,1\n',
            b'20.000,BRAKE,PID,0,39.4000,40.2000,40.1000,0.8000,0.1000,0.0400,0.0600,0.0000,49.40,652,0\n',
            b'40.000,BRAKE,PID,0,38.8000,38.8000,38.9000,0.0000,0.2000,0.0600,0.1400,0.0000,49.80,653,0\n',
            b'#END_CSV\n', f'#RESULT {result}\n'.encode()]


class CapturePidTests(unittest.TestCase):
    CMD = 'runpid 200 3000 1000 8000'
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = Path(self.temp.name)/'pid.csv'
        self.addCleanup(self.temp.cleanup)
    def run_capture(self, lines, command=CMD):
        clock = Clock()
        serial = Serial(lines, clock)
        try:
            capture.capture(serial, self.path, command, timeout=.05, clock=clock)
        finally:
            self.assertEqual(serial.sent[-1], b'x\n')
        return serial
    def rejects(self, lines, command=CMD):
        with tempfile.TemporaryDirectory() as folder:
            self.path = Path(folder)/'pid.csv'
            with self.assertRaises(capture.CaptureError): self.run_capture(lines, command)
            self.assertFalse(self.path.exists())
    def test_valid_runpid_capture_and_command_sent(self):
        serial = self.run_capture(transcript_pid())
        self.assertEqual(serial.sent[0], b'runpid 200 3000 1000 8000\n')
        self.assertEqual(capture.validate_csv(self.path, self.CMD), 4)
        meta = json.loads(self.path.with_suffix('.capture.json').read_text())
        self.assertEqual((meta['status'], meta['result']), ('complete', 'w_final_alcanzada'))
    def test_other_valid_results(self):
        for result in ['parada_detectada', 'duracion_maxima']:
            with self.subTest(result=result), tempfile.TemporaryDirectory() as folder:
                self.path = Path(folder)/'pid.csv'
                self.run_capture(transcript_pid(result))
    def test_w_final_is_not_valid_for_step(self):
        lines = transcript('w_final_alcanzada')
        with self.assertRaises(capture.CaptureError): self.run_capture_step(lines)
    def run_capture_step(self, lines):
        clock = Clock()
        capture.capture(Serial(lines, clock), self.path, 'step 150 .4 20 20 1000', timeout=.05, clock=clock)
    def test_failed_runs_never_published(self):
        for reason in ['cancelado: motor PWM=0, freno liberado',
                       'error_sin_velocidad_inicial: w <= w_stop, no se inicia el PID',
                       'error_buffer_serial: captura incompleta']:
            with self.subTest(reason=reason): self.rejects(transcript_pid(reason))
    def test_header_and_command_must_agree(self):
        self.rejects(transcript_pid(), 'step 150 .4 20 20 1000')      # CSV de 15 columnas con comando step
        self.rejects(transcript(), self.CMD)                           # CSV de 9 columnas con comando runpid
    def test_pre_must_match_command_pwm(self):
        self.rejects(transcript_pid(), 'runpid 180 3000 1000 8000')
    def test_corrupt_pid_rows(self):
        base = transcript_pid()
        bad_rows = [
            b'20.000,BRAKE,PID,0,39.4000,40.2000,40.1000,0.9000,0.1000,0.0400,0.0600,0.0000,49.40,652,0\n',  # error mal calculado
            b'20.000,BRAKE,PID,0,39.4000,40.2000,40.1000,0.8000,0.5000,0.0400,0.0600,0.0000,49.40,652,0\n',  # u != P+I+D
            b'20.000,BRAKE,PID,0,40.4000,40.2000,40.1000,-0.2000,0.0000,0.0000,0.0000,0.0000,44.00,652,1\n', # ref sube
            b'20.000,BRAKE,OPEN,0,39.4000,40.2000,40.1000,0.8000,0.1000,0.0400,0.0600,0.0000,49.40,652,0\n', # modo
            b'20.000,BRAKE,PID,1,39.4000,40.2000,40.1000,0.8000,0.1000,0.0400,0.0600,0.0000,49.40,652,0\n', # motor en freno
            b'20.000,BRAKE,PID,0,39.4000,40.2000,40.1000,0.8000,0.1000,0.0400,0.0600,0.0000,49.40,652,2\n', # flag u=1 con u=.1
            b'20.000,BRAKE,PID,0,39.4000,40.2000,40.1000,0.8000,0.1000,0.0400,0.0600,0.0000,49.40,652,32\n', # flags fuera de rango
            b'20.000,BRAKE,PID,0,39.4000,nan,40.1000,0.8000,0.1000,0.0400,0.0600,0.0000,49.40,652,0\n',     # nan
            b'20.000,BRAKE,PID,0,39.4000,40.2000,40.1000,0.8000,0.1000,0.0400,0.0600,0.0000,49.40,600,0\n', # pulsos retroceden
        ]
        for bad in bad_rows:
            with self.subTest(row=bad[:60]):
                lines = list(base); lines[4] = bad
                self.rejects(lines)
    def test_first_brake_row_starts_at_measured_speed(self):
        lines = transcript_pid()
        lines[3] = b'0.000,BRAKE,PID,0,45.0000,40.0000,40.0283,-5.0000,0.0000,0.0000,0.0000,0.0000,44.00,651,1\n'
        self.rejects(lines)
    def test_pre_rows_must_have_zero_pid_fields(self):
        lines = transcript_pid()
        lines[2] = b'-20.000,PRE,OPEN,200,40.0000,40.0000,40.0283,0.0000,0.0000,0.0000,0.0000,0.0000,44.00,650,0\n'
        self.rejects(lines)
    def test_abs_flag_skips_sum_check(self):
        lines = transcript_pid()
        lines[4] = b'20.000,BRAKE,PID,0,39.4000,40.2000,40.1000,0.8000,0.6000,0.0400,0.0600,0.0000,52.00,652,4\n'
        self.run_capture(lines)    # con ABS activo u puede diferir de P+I+D
    def test_saturation_flags_accepted(self):
        lines = transcript_pid()
        lines[4] = b'20.000,BRAKE,PID,0,39.4000,50.0000,50.0000,10.6000,1.0000,0.6000,0.9000,0.0000,53.00,652,18\n'
        self.run_capture(lines)
    def test_timeouts_for_runpid(self):
        wait, data = capture.capture_timeouts('runpid 1 600000 600000 600000')
        self.assertGreater(wait, 600); self.assertGreater(data, 1200)
        for command in ['runpid', 'runpid 200 3000 1000', 'runpid 0 3000 1000 8000', 'runpid 256 3000 1000 8000',
                        'runpid 200 19 1000 8000', 'runpid 200 3000 1000 19', 'runpid 200 3000 1000 600001',
                        'runpid 200 3000 1000 8000 1', 'runpid 200 a 1000 8000', 'runpid -1 3000 1000 8000']:
            with self.subTest(command=command), self.assertRaises(ValueError): capture.capture_timeouts(command)
    def test_setup_commands_are_sent_before_runpid_in_order(self):
        clock = Clock()
        serial = Serial(transcript_pid(), clock)
        capture.capture(serial, self.path, self.CMD, timeout=.05, clock=clock,
                        setup=['pid 0.02 0 0 0.05', ' ref 40 5 '])
        self.assertEqual(serial.sent[:3], [b'pid 0.02 0 0 0.05\n', b'ref 40 5\n', b'runpid 200 3000 1000 8000\n'])
        meta = json.loads(self.path.with_suffix('.capture.json').read_text())
        self.assertEqual(meta['setup'], ['pid 0.02 0 0 0.05', 'ref 40 5'])
    def test_setup_rejected_by_firmware_aborts_and_stops(self):
        lines = [b'#ERR uso: pid <kp 0..100> <ki 0..1000> <kd 0..100> <tf 0..5 s>\n']
        clock = Clock()
        serial = Serial(lines, clock)
        with self.assertRaises(capture.CaptureError):
            capture.capture(serial, self.path, self.CMD, timeout=.05, clock=clock, setup=['pid 999 0 0 0'])
        self.assertEqual(serial.sent[-1], b'x\n')
        self.assertFalse(self.path.exists())
    def test_invalid_setup_commands(self):
        for bad in [['m 100'], ['step 150 0 3000 1000 10000'], ['runpid 200 3000 1000 8000'], [''], ['pid 1 1 1 1', 'x'],
                    ['pid 1 1 1 1x'], ['ref 40 5\x00'], ['pid 1 1 1 \u00e9']]:
            with self.subTest(setup=bad), self.assertRaises(ValueError):
                capture.check_setup(bad)
        self.assertEqual(capture.check_setup(['PID 1 1 1 1', 'cfg mp 4000']), ['PID 1 1 1 1', 'cfg mp 4000'])
        with self.assertRaises(ValueError):
            capture.capture(Serial([], Clock()), self.path, self.CMD, setup=['m 100'])
        self.assertFalse(self.path.with_suffix('.partial.csv').exists())
    def test_fuzz_protocol_pid_never_completes_without_result(self):
        rng = random.Random(1111)
        for _ in range(10000):
            protocol = capture.CaptureProtocol(io.StringIO(), capture.HEADER_PID, capture.SUCCESS_RESULTS_PID)
            lines = [rng.choice(['#START_CSV', '#END_CSV', capture.HEADER_PID, capture.HEADER, 'garbage', '#ERR nope', '#WARN nope']) for _ in range(rng.randrange(1, 20))]
            try:
                for line in lines: protocol.accept(line)
            except capture.CaptureError: pass
            self.assertNotEqual(protocol.state, 'DONE')


if __name__ == '__main__': unittest.main()
