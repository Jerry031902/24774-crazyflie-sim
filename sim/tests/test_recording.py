"""File-capture regressions independent of Isaac Sim/GPU."""
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from record_mission import png_complete, wait_for_capture


class CaptureWriteTests(unittest.TestCase):
    def test_completed_future_waits_for_partial_png_to_finish(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'frame.png'
            task = Mock()
            task.done.return_value = True
            updates = []

            def render():
                updates.append(1)
                if len(updates) == 1:
                    path.write_bytes(b'\x89PNG\r\n\x1a\n')
                    self.assertFalse(png_complete(path))
                elif len(updates) == 3:
                    with path.open('ab') as image:
                        image.write(b'\x00\x00\x00\x00IEND\xaeB`\x82')

            wait_for_capture(task, path, render, lambda: True)
            self.assertEqual(len(updates), 3)
            task.result.assert_called_once()
            task.cancel.assert_not_called()

    def test_missing_file_times_out_even_if_future_completed(self):
        with tempfile.TemporaryDirectory() as folder:
            task = Mock()
            task.done.return_value = True
            render = Mock()
            with patch('record_mission.time.monotonic', side_effect=[0., 0., .5, 1.1]):
                with self.assertRaisesRegex(RuntimeError, 'image write did not complete'):
                    wait_for_capture(task, Path(folder)/'missing.png', render, lambda: True, timeout=1.)
            self.assertEqual(render.call_count, 2)

    def test_capture_exception_is_not_hidden_as_write_timeout(self):
        task = Mock()
        task.done.return_value = True
        task.result.side_effect = RuntimeError('GPU capture failed')
        with self.assertRaisesRegex(RuntimeError, 'GPU capture failed'):
            wait_for_capture(task, Path('/unused.png'), Mock(), lambda: True)


if __name__ == '__main__':
    unittest.main()
