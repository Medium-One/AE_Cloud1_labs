import sys
import logging
import hashlib

def get_sha256_hash(data):
        s = hashlib.sha256()

        def slicen(s, n, truncate=False):
            assert n > 0
            while len(s) >= n:
                yield s[:n]
                s = s[n:]
            if len(s) and not truncate:
                yield s

        first = False
        if len(data) % 64:
            logging.debug("{} byte padding required".format(64 - (len(data) % 64)))
        buf = data + '\0' * (64 - (len(data) % 64))
        reverse = True
        CHUNK_SIZE = 1536
        for chunk in slicen(buf, CHUNK_SIZE):
            newchunk = []
            for word in slicen(chunk, 4):
                newchunk.extend(reversed(word) if reverse else word)
            if len(newchunk) != CHUNK_SIZE:
                logging.debug('small chunk: {}'.format(len(newchunk)))
            if first:
                for word in slicen(newchunk, 4):
                    logging.debug('0x{}'.format(''.join(['{:02x}'.format(ord(x)) for x in word])))
                first = False
            s.update(''.join(newchunk).encode())
        return s.hexdigest()

if __name__ == '__main__':
    if sys.version_info[0] >= 3:
        print(get_sha256_hash(open(sys.argv[1], 'r', newline='').read()))
    else:
        print(get_sha256_hash(open(sys.argv[1], 'r').read()))
