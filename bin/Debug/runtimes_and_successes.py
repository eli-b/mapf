import sys  
import os.path
import csv
from collections import defaultdict
from collections import Counter
from functools import partial
from pprint import pprint
import numpy as np
import matplotlib.pyplot as plt

input_paths = sys.argv[1:]

# 1. Calculate success rate - a solver needs to solve 80% of the problems in a category to have its its runtime averaged at all.
readers = [csv.DictReader(open(input_path, 'rb')) for input_path in input_paths]

solver_successes_per_num_of_agents = defaultdict(Counter)
solver_run_count_per_num_of_agents = defaultdict(Counter)

for i, reader in enumerate(readers):
    print "Reading input file ", i
    success_fieldnames = {fieldname for fieldname in reader.fieldnames if fieldname.endswith('Success')}
    solution_cost_fieldnames = {fieldname for fieldname in reader.fieldnames if fieldname.endswith(" Solution Cost")}
    for row in reader:
        try:
            num_of_agents = int(row['Num Of Agents'])
            solvers_run = [col_name[:-len(" Solution Cost")] for col_name, col_val in row.iteritems() \
                       if col_name in solution_cost_fieldnames if col_val != "irrelevant"] # Algs that were actually run
            solvers_that_succeeded = [col_name[:-len(" Success")] for col_name, col_val in row.iteritems() \
                                      if col_name in success_fieldnames if int(col_val) == 1]
        except (TypeError, ValueError) as e:
            print "Problem in row: num agents=", row["Num Of Agents"], " instance id=", row["Instance Id"]
            raise
        solver_run_count_per_num_of_agents[num_of_agents].update(solvers_run) # Add the counts of the items in solvers
        solver_successes_per_num_of_agents[num_of_agents].update(solvers_that_succeeded)

solver_success_rate_per_num_of_agents = defaultdict(Counter)        
# 2. Average the results:
for num_of_agents, success_counts in solver_successes_per_num_of_agents.iteritems():
    for solver, success_count in success_counts.iteritems():
        solver_success_rate_per_num_of_agents[num_of_agents][solver] = float(success_count) / solver_run_count_per_num_of_agents[num_of_agents][solver]
     
# 3. Print success rates per category
for num_of_agents, success_rates in sorted(solver_success_rate_per_num_of_agents.iteritems()):
    print 'Success rate for %d agents' % (num_of_agents, )
    pprint(sorted(success_rates.iteritems(), key=lambda x:x[1], reverse=True))

# 4. Calculate average runtime for solvers that succeeded iin solving at least 80% of the problems.    
readers = [csv.DictReader(open(input_path, 'rb')) for input_path in input_paths]

solver_relevant_runtimes_per_num_of_agents = defaultdict(Counter)
solver_relevant_run_count_per_num_of_agents = defaultdict(Counter)

for i, reader in enumerate(readers):
    print "Reading input file ", i
    runtime_fieldnames = [fieldname for fieldname in reader.fieldnames if fieldname.endswith('Runtime') and not fieldname.endswith('Average Runtime')]
    success_fieldnames = [fieldname for fieldname in reader.fieldnames if fieldname.endswith('Success')]
    solution_cost_fieldnames = {fieldname for fieldname in reader.fieldnames if fieldname.endswith(" Solution Cost")}
    for row in reader:
        try:
            num_of_agents = int(row['Num Of Agents'])
            solvers_run = {col_name[:-len(" Solution Cost")] for col_name, col_val in row.iteritems() \
                           if col_name in solution_cost_fieldnames if col_val != "irrelevant"}
            relevant_solvers_in_category = {solver_name for solver_name in solvers_run \
                                            if solver_success_rate_per_num_of_agents[num_of_agents][solver_name] > 0.1} # Algs that were actually run and succeeded in solving at least 0.1 of the problems in this category
            solvers_that_succeeded = {col_name[:-len(" Success")] for col_name, col_val in row.iteritems() \
                                      if col_name in success_fieldnames if int(col_val) == 1}
            relevant_solvers_that_succeeded = {solver_name for solver_name in solvers_that_succeeded \
                                              if solver_name in relevant_solvers_in_category}
            solvers_and_runtimes = {col_name[:-len(" Runtime")]:float(col_val) for col_name, col_val in row.iteritems() \
                                    if col_name in runtime_fieldnames}
            relevant_solvers_and_runtimes = {solver_name:runtime for solver_name, runtime in solvers_and_runtimes.iteritems() \
                                             if solver_name in relevant_solvers_in_category}
            
        except (TypeError, ValueError) as e:
            print "Problem in row: num agents=", row["Num Of Agents"], " instance id=", row["Instance Id"]
            raise
        
        if relevant_solvers_that_succeeded != relevant_solvers_in_category:
            print "Not using runtimes of row: num agents={num_agents}, id={id} - {failures} didn't solve it".format(num_agents=row["Num Of Agents"], id=row["Instance Id"], failures=relevant_solvers_in_category - relevant_solvers_that_succeeded)
            continue # If any relevant algorithm failed, skip the problem to avoid making an unfair comparison.
        solver_relevant_runtimes_per_num_of_agents[num_of_agents].update(relevant_solvers_and_runtimes)
        solver_relevant_run_count_per_num_of_agents[num_of_agents].update(relevant_solvers_and_runtimes.iterkeys())

solver_average_runtimes_per_num_of_agents = defaultdict(partial(defaultdict, lambda : None)) # Plot average runtime for solvers that weren't averaged for that category as missing data
# Average the results:
for num_of_agents, solver_runtimes in solver_relevant_runtimes_per_num_of_agents.iteritems():
    for solver, runtime in solver_runtimes.iteritems():
        solver_average_runtimes_per_num_of_agents[num_of_agents][solver] = runtime / solver_relevant_run_count_per_num_of_agents[num_of_agents][solver]

# Print relevant average runtimes per category
print
print
print
for num_of_agents, solver_averages in sorted(solver_average_runtimes_per_num_of_agents.iteritems()):
    print 'Average runtimes for %d agents' % (num_of_agents, )
    pprint(sorted(solver_averages.iteritems(), key=lambda x:x[1]))

sorted_solver_names = list(sorted({fieldname[:-len(" Solution Cost")] for fieldname in reader.fieldnames if fieldname.endswith(" Solution Cost") for reader in readers}))
sorted_num_of_agents = np.array(list(sorted(solver_success_rate_per_num_of_agents.iterkeys())))

# Plot the results
fig = plt.figure(1) # Figure ID 1
if (len(input_paths) == 1): # Use input file name as window title and figure title
    title = os.path.splitext(os.path.basename(sys.argv[1]))[0]
    # Set window title
    fig.canvas.set_window_title(title)
    # Set figure title
    fig.suptitle(title, size="x-large")
plt.subplot(1, 2, 1) # First of two side-by-side figures
for solver in sorted_solver_names:
    plt.plot(sorted_num_of_agents, np.array([100*solver_success_rate_per_num_of_agents[num_agents][solver] for num_agents in sorted_num_of_agents]), "D-", label=solver)
plt.title('Success Rates')
plt.xlabel('Number Of Agents')
plt.ylabel('Success Rate (%)')
plt.yscale('linear')
plt.legend(loc='lower left', )
plt.ylim(0, 100)
plt.subplot(1, 2, 2) # Second of two side-by-side figures
for solver in sorted_solver_names:
    plt.plot(sorted_num_of_agents, np.array([solver_average_runtimes_per_num_of_agents[num_agents][solver] for num_agents in sorted_num_of_agents]), "D-", label=solver)
plt.title('Average Runtimes')
plt.xlabel('Number Of Agents')
plt.ylabel('Average Runtime (ms)')
plt.yscale('log')
plt.legend(loc='upper left', )
plt.ylim(0, 250000) # Hack to leave just enough room at the top of the plot for the legend.
                    # Warning: Turns off autoscaling for the axis
plt.show()
