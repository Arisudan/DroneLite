"""map_feed.FrameShare copies frames into shared memory ONLY while a reader (the map pipeline) keeps its heartbeat fresh."""
import struct
import time

import numpy as np

from nimbus_gcs import map_feed as M


def test_frames_are_copied_only_while_the_map_reads_them():
    n = [0]

    def frame():
        n[0] += 1; return np.full((48, 64, 3), n[0] % 255, np.uint8), time.time()

    fs = M.FrameShare(frame)
    try:
        time.sleep(0.3); assert fs.frames == 0 and not fs.reader_alive()                     # no map running: nothing copied
        struct.pack_into("<d", fs.buf, M.READER_HB, time.time())                             # what o4_map_node's FrameReader.read() writes
        time.sleep(0.3); assert fs.reader_alive() and fs.frames > 5
        h, w = struct.unpack_from("<II", fs.buf, 8); assert (h, w) == (48, 64)
        struct.pack_into("<d", fs.buf, M.READER_HB, time.time() - 10)                        # the map stopped
        time.sleep(0.15); k = fs.frames; time.sleep(0.3); assert fs.frames == k
    finally:
        fs.close()
