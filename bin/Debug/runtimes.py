import sys  
import os.path
import csv
from collections import defaultdict
from collections import Counter
from functools import partial
from pprint import pprint
import numpy as np
import pylab

input_path = sys.argv[1]
input_filename = os.path.splitext(input_path)[0]
output_path = input_filename + ' with analysis' + '.csv'

reader = csv.DictReader(open(input_path, 'rb'))

target_fieldnames = [fieldname for fieldname in reader.fieldnames if fieldname.endswith('Runtime') and not fieldname.endswith('Average Runtime')]

runtimes = defaultdict(float)
runtimes_per_num_of_agents = defaultdict(partial(defaultdict, float))
rows_per_num_of_agents = Counter()

for row in reader:
    solvers_and_runtimes = {col_name[:-len(" Success")]:float(col_val) for col_name, col_val in row.iteritems() \
                            if col_name in target_fieldnames}
    for solver, runtime in solvers_and_runtimes.iteritems():
        runtimes[solver] += runtime
        num_of_agents = int(row['Num Of Agents'])
        runtimes_per_num_of_agents[num_of_agents][solver] += runtime
        rows_per_num_of_agents[num_of_agents] += 1
        
    #if solvers_and_runtimes['Basic CBS/A*/SIC with OpenList'] > solvers_and_runtimes['Basic CBS/A*+OD/SIC with OpenList']:
    #    print row['Grid Name'], row['Num Of Agents'], row['Instance Id'], solvers_and_runtimes['Basic CBS/A*/SIC with OpenList'] - solvers_and_runtimes['Basic CBS/A*+OD/SIC with OpenList']

print 'Overall runtimes:'
pprint(sorted(runtimes.iteritems(), key=lambda x:x[1]))
for num_of_agents in sorted(runtimes_per_num_of_agents):
    rtimes = runtimes_per_num_of_agents[num_of_agents]
    print 'Runtimes for %d agents' % (num_of_agents, )
    pprint(sorted(rtimes.iteritems(), key=lambda x:x[1]))
    
runtimes_sorted_by_num_of_agents = sorted(runtimes_per_num_of_agents.iteritems())
data = []
for agents_num, runtimes in runtimes_sorted_by_num_of_agents:
    runtimes_sorted_by_alg_name = sorted(runtimes.iteritems())
    row = [agents_num]
    row.extend((runtime/rows_per_num_of_agents[agents_num] for alg, runtime in runtimes_sorted_by_alg_name))
    data.append(row)
data = np.array(data)
x = data[:, 0]
for i, alg_name in enumerate(runtimes_sorted_by_num_of_agents[0][1].iterkeys()):
    pylab.plot(x, data[:, i+1], "D-", label=alg_name)
pylab.title('Average Runtimes')
pylab.xlabel('Number of agents')
pylab.ylabel('Average Runtime (ms)')
#pylab.yscale('linear')
#pylab.ylim(0, 300000)
pylab.legend(loc='upper left')
#pylab.legend(loc='lower left')
pylab.show()
