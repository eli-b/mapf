import sys  
import os.path
import csv
from collections import defaultdict
from collections import Counter
from functools import partial
from pprint import pprint
import numpy as np
import pylab

input_paths = sys.argv[1:]

readers = (csv.DictReader(open(input_path, 'rb')) for input_path in input_paths)

runtimes = defaultdict(float)
runtimes_per_num_of_agents = defaultdict(partial(defaultdict, float))
solver_run_count_per_num_of_agents = defaultdict(Counter)
total_solver_run_counts = Counter()

for i, reader in enumerate(readers):
    print "Reading input file ", i
    target_fieldnames = [fieldname for fieldname in reader.fieldnames if fieldname.endswith('Runtime') and not fieldname.endswith('Average Runtime')]
    for row in reader:
        try:
            solvers_and_runtimes = {col_name[:-len(" Runtime")]:float(col_val) for col_name, col_val in row.iteritems() \
                                    if col_name in target_fieldnames}
        except (TypeError, ValueError) as e:
            print "Problem in row: num agents=", row["Num Of Agents"], " instance id=", row["Instance Id"]
            raise
        for solver, runtime in solvers_and_runtimes.iteritems():
            runtimes[solver] += runtime
            total_solver_run_counts[solver] += 1
            num_of_agents = int(row['Num Of Agents'])
            runtimes_per_num_of_agents[num_of_agents][solver] += runtime
            solver_run_count_per_num_of_agents[num_of_agents][solver] += 1
            
        #if solvers_and_runtimes['Basic CBS/A*/SIC with OpenList'] > solvers_and_runtimes['Basic CBS/A*+OD/SIC with OpenList']:
        #    print row['Grid Name'], row['Num Of Agents'], row['Instance Id'], solvers_and_runtimes['Basic CBS/A*/SIC with OpenList'] - solvers_and_runtimes['Basic CBS/A*+OD/SIC with OpenList']
        
# Average the results:
for solver, runtime in runtimes.iteritems():
    runtimes[solver] = runtime / total_solver_run_counts[solver]
    
for num_of_agents in runtimes_per_num_of_agents:
    for solver, runtime in runtimes_per_num_of_agents[num_of_agents].iteritems():
        runtimes_per_num_of_agents[num_of_agents][solver] = runtime / solver_run_count_per_num_of_agents[num_of_agents][solver]

# Not printing overall runtimes as the comparison might not be fair - the slower algs aren't run on the tougher problems

for num_of_agents in sorted(runtimes_per_num_of_agents):
    rtimes = runtimes_per_num_of_agents[num_of_agents]
    print 'Average runtimes for %d agents' % (num_of_agents, )
    pprint(sorted(rtimes.iteritems(), key=lambda x:x[1]))
    
runtimes_sorted_by_num_of_agents = list(sorted(runtimes_per_num_of_agents.iteritems()))
sorted_solver_names = list(sorted(runtimes.iterkeys()))

lines = []
for solver in sorted_solver_names:
    data = []
    for agents_num, runtimes_for_num_agents in runtimes_sorted_by_num_of_agents:
        if solver in runtimes_for_num_agents:
            solver_runtime_for_num_agents = runtimes_for_num_agents[solver]
            data_row = [agents_num, solver_runtime_for_num_agents]
            data.append(data_row)
    data = np.array(data)
    x = data[:, 0]
    l = pylab.plot(x, data[:, 1], "D-", label=solver)
    lines.append(l)
pylab.title('Average Runtimes')
pylab.xlabel('Number of agents')
pylab.ylabel('Average Runtime (ms)')
pylab.yscale('linear')
pylab.legend(loc='upper left', )#bbox_to_anchor=(1.05, 1), borderaxespad=0.)
#pylab.yscale('log')
#pylab.legend(loc='lower left', )#bbox_to_anchor=(1.05, 1), borderaxespad=0.)
#pylab.ylim(0, 300000)

#handles, labels = pylab.gca().get_legend_handles_labels()
#pylab.figlegend(handles, labels, loc='upper left')
#pylab.legend(loc='lower left')
pylab.show()
