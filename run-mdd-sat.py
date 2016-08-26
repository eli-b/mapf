import subprocess
import multiprocessing
import os
import argparse
import time
import glob
import signal
import sys

def main():
    pass
    
if __name__ == '__main__':
    def myworker(filepath):
        #if 'maze' not in filepath:
        #    return
        if 'output' in filepath:
            return
        output_path = '{}_output.txt'.format(filepath)
        if os.path.exists(output_path):
            return
        if not os.path.exists(filepath):
            return
        command = './solver_reLOC --output-file={1} --bgu-input={0} --cost-limit=65536 --layer-limit=65536 --makespan-limit=65536 --minisat-timeout=300 --total-timeout=300 --encoding=mdd > /dev/null'.format(filepath, output_path)
        print 'running {}'.format(command)
        try:
            start = time.time()
            subprocess.check_call(command, shell=True)
            end = time.time()
            with open(output_path, 'a') as f:
                f.write('\nTotal time (seconds): {}'.format(end - start))
        except:
            #pool.terminate()
            raise
            
    def gen():
        for agents in xrange(5, 85, 5):
            for i in xrange(100):
                #maze_index = int(sys.argv[0])
                #yield '/home/ubuntu/maze512-1-{}-{}-{}'.format(maze_index, agents, i)
                #yield '/home/ubuntu/maze512-2-{}-{}-{}'.format(maze_index, agents, i)
                if i % 3 == int(sys.argv[0):
                    yield '/home/ubuntu/ost003d-{}-{}'.format(agents, i)
                    yield '/home/ubuntu/den520d-{}-{}'.format(agents, i)
                    yield '/home/ubuntu/brc202d-{}-{}'.format(agents, i)
    
    original_sigint_handler = signal.signal(signal.SIGINT, signal.SIG_IGN)
    pool = multiprocessing.Pool()
    signal.signal(signal.SIGINT, original_sigint_handler)
    try:
        res = pool.map_async(myworker, gen())
        res.get(9999999999999999999999999)
    except KeyboardInterrupt:
        pool.terminate()
    else:
        pool.close()
    pool.join()
