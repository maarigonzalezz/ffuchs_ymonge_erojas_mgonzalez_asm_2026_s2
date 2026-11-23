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


if __name__ == '__main__': unittest.main()
