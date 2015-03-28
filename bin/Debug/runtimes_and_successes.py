import sys  
import os.path
import csv
from collections import defaultdict
from collections import Counter
from functools import partial
import itertools
from pprint import pprint
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import re

input_paths = sys.argv[1:]

# 1. Calculate success rate -
#    a solver needs to solve 30% of the problems in a category to have its its runtime averaged at all.
min_success_to_consider = 0.0
readers = [csv.DictReader(open(input_path, 'rb')) for input_path in input_paths]
#pd_data = pd.concat((pd.readcsv(path) for path in input_paths))

solver_successes_per_num_of_agents = defaultdict(Counter)
solver_run_count_per_num_of_agents = defaultdict(Counter)
solvers = set()
# pd variant:
#category_name = "Num Of Agents"
#by_num_agents = pd_data.group_by(category_name)

#solver_run_count_per_num_of_agents = 

for i, reader in enumerate(readers):
    print "Reading input file ", i
    success_fieldnames = {fieldname for fieldname in reader.fieldnames if fieldname.endswith('Success')}
    solution_cost_fieldnames = {fieldname for fieldname in reader.fieldnames if fieldname.endswith(" Solution Cost")}
    for row in reader:
        try:
            num_of_agents = int(row['Num Of Agents'])
            solvers_run = [col_name[:-len(" Solution Cost")] for col_name, col_val in row.iteritems() \
                           if col_name in solution_cost_fieldnames if col_val != "irrelevant"] # Algs that were actually run
            solvers.update(solvers_run)
            solvers_that_succeeded = [col_name[:-len(" Success")] for col_name, col_val in row.iteritems() \
                                      if col_name in success_fieldnames if int(col_val) == 1]
        except (TypeError, ValueError) as e:
            print "Problem in row: num agents=", row["Num Of Agents"], " instance id=", row["Instance Id"]
            #raise
            continue
        solver_run_count_per_num_of_agents[num_of_agents].update(solvers_run) # Add the counts of the items in solvers
        solver_successes_per_num_of_agents[num_of_agents].update(solvers_that_succeeded)

solver_success_rate_per_num_of_agents = defaultdict(Counter)        
# 2. Average the results:
for num_of_agents, success_counts in solver_successes_per_num_of_agents.iteritems():
    for solver, success_count in success_counts.iteritems():
        solver_success_rate_per_num_of_agents[num_of_agents][solver] = float(success_count) / solver_run_count_per_num_of_agents[num_of_agents][solver]
     
# 3. Print success rates per category
parameters_pat = re.compile(r"\d+", re.DOTALL & re.VERBOSE)
sorted_solver_names = list(sorted(
                                  (list(sorted(solvers,
                                   key=lambda name: [int(param) for param in parameters_pat.findall(name.replace(r'$\infty$', '99999999999999999'))]))), # first order by numeric params
                                   key=lambda name:parameters_pat.subn('', name)[0].replace(r'$\infty$', ''))) # Then by name
#sorted_solver_names = ["MA-CBS+BP", "MA-CBS+ID", "ICTS+ID", "EPEA*+ID", "MA-CBS", ] # "MA-CBS+BP2",] # Allows manually choosing the order of solvers in the legend
#sorted_solver_names = ["MA-CBS+BP2", "MA-CBS+BP1", "MA-CBS"] # Allows manually choosing the order of solvers in the legend
#sorted_solver_names = ["ICBS(10)+ID", "ICTS+ID", "EPEA*+ID", ] # Allows manually choosing the order of solvers in the legend
#sorted_solver_names = ["CBS+IMP1+IMP2", "CBS+IMP1", "CBS+IMP2", "CBS"]
sorted_solver_names = ["MA-CBS(5)+IMP3", "MA-CBS(5)",]
#sorted_solver_names = ["MA-CBS(256)", "EPEA*", "ICBS(256)", "ICTS", "CBS+IMP1+IMP2", "CBS+IMP1", "CBS"]
#sorted_solver_names = ["MA-CBS(5)", "MA-CBS(50)", "MA-CBS(5)+IMP3", "ICBS(5) (full restart)"]
#sorted_solver_names = [ "MA-CBS(64)+IMP3", "MA-CBS(64)"]  # "CBS", "ICTS", "EPEA*",
sorted_num_of_agents = np.array(list(sorted(solver_success_rate_per_num_of_agents.iterkeys())))

for num_of_agents, success_rates in sorted(solver_success_rate_per_num_of_agents.iteritems()):
    print 'Success rate for %d agents' % (num_of_agents, )
    pprint(sorted(success_rates.iteritems(), key=lambda x:x[1], reverse=True))
# TeX table:
print "Success Rates:"
print '|r|' + '|'.join(('r' for solver in sorted_solver_names)) + '|'
print "Agents & " + ' & '.join(sorted_solver_names) + r'\\'
for num_of_agents, solver_success_rates in sorted(solver_success_rate_per_num_of_agents.iteritems()):
    max_val = max(solver_success_rates.itervalues())
    print "{:>2d} & ".format(num_of_agents) + \
        ' & '.join("{:^15s}".format(("" if solver_success_rates[solver] != max_val else r"\bf{") + \
                   ("{:.0%}".format(solver_success_rates[solver]) if solver_success_rates[solver] is not None else "N/A") + \
                   ("" if solver_success_rates[solver] != max_val else r"}"))  \
                   for solver in sorted_solver_names) + \
        r'\\'

# 4. Calculate average runtime for solvers that succeeded iin solving at least 80% of the problems.    
readers = [csv.DictReader(open(input_path, 'rb')) for input_path in input_paths]

solver_relevant_runtimes_per_num_of_agents = defaultdict(Counter)
solver_relevant_run_count_per_num_of_agents = defaultdict(Counter)
solver_relevant_generated_per_num_of_agents = defaultdict(Counter)
solver_relevant_lookaheads_per_num_of_agents = defaultdict(Counter)
solver_relevant_adoptions_per_num_of_agents = defaultdict(Counter)
solver_relevant_conflicts_solved_with_adoption_per_num_of_agents = defaultdict(Counter)
solver_relevant_expanded_per_num_of_agents_per_num_of_agents = defaultdict(Counter)
solver_relevant_nodes_with_goal_cost_per_num_of_agents = defaultdict(Counter)
solver_relevant_expanded_per_num_of_agents = defaultdict(Counter)
num_averaged_problems_per_num_of_agents = Counter()

for i, reader in enumerate(readers):
    print "Reading input file ", i
    runtime_fieldnames = [fieldname for fieldname in reader.fieldnames if fieldname.endswith('Runtime') and not fieldname.endswith('Average Runtime')]
    success_fieldnames = [fieldname for fieldname in reader.fieldnames if fieldname.endswith('Success')]
    solution_cost_fieldnames = {fieldname for fieldname in reader.fieldnames if fieldname.endswith(" Solution Cost")}
    generated_fieldnames = {fieldname for fieldname in reader.fieldnames if fieldname.endswith(" Generated (HL)")}
    lookahead_fieldnames = {fieldname for fieldname in reader.fieldnames if fieldname.endswith(" Look Ahead Nodes Created (HL)")}
    adoptions_fieldnames = {fieldname for fieldname in reader.fieldnames if fieldname.endswith(" Adoptions (HL)")}
    conflicts_solved_with_adoption_fieldnames = {fieldname for fieldname in reader.fieldnames if fieldname.endswith(" Conflicts Bypassed With Adoption (HL)")}
    nodes_expanded_with_goal_cost_fieldnames = {fieldname for fieldname in reader.fieldnames if fieldname.endswith(" Nodes Expanded With Goal Cost (HL)")}
    expanded_fieldnames = {fieldname for fieldname in reader.fieldnames if fieldname.endswith(" Expanded (HL)")}
    
    for row in reader:
        try:
            num_of_agents = int(row['Num Of Agents'])
            solvers_run = {col_name[:-len(" Solution Cost")] for col_name, col_val in row.iteritems() \
                           if col_name in solution_cost_fieldnames if col_val != "irrelevant"}
            relevant_solvers_in_category = {solver_name for solver_name in solvers_run \
                                            if solver_success_rate_per_num_of_agents[num_of_agents][solver_name] > min_success_to_consider} # Algs that were actually run and succeeded in solving at least min_success_to_consider of the problems in this category
            solvers_that_succeeded = {col_name[:-len(" Success")] for col_name, col_val in row.iteritems() \
                                      if col_name in success_fieldnames if int(col_val) == 1}
            relevant_solvers_that_succeeded = {solver_name for solver_name in solvers_that_succeeded \
                                               if solver_name in relevant_solvers_in_category}
            solvers_and_runtimes = {col_name[:-len(" Runtime")]:float(col_val) for col_name, col_val in row.iteritems() \
                                    if col_name in runtime_fieldnames}
            solvers_and_generated = {col_name[:-len(" Generated (HL)")]:float(col_val) for col_name, col_val in row.iteritems() \
                                    if col_name in generated_fieldnames}
            solvers_and_lookaheads = {col_name[:-len(" Look Ahead Nodes Created (HL)")]:float(col_val) for col_name, col_val in row.iteritems() \
                                    if col_name in lookahead_fieldnames}
            solvers_and_adoptions = {col_name[:-len(" Adoptions (HL)")]:float(col_val) for col_name, col_val in row.iteritems() \
                                    if col_name in adoptions_fieldnames}
            solvers_and_conflicts_solved_with_adoption = {col_name[:-len(" Nodes Expanded With Goal Cost (HL)")]:float(col_val) for col_name, col_val in row.iteritems() \
                                    if col_name in conflicts_solved_with_adoption_fieldnames}
            solvers_and_nodes_with_goal_cost = {col_name[:-len(" Conflicts Bypassed With Adoption (HL)")]:float(col_val) for col_name, col_val in row.iteritems() \
                                    if col_name in nodes_expanded_with_goal_cost_fieldnames}
            solvers_and_expanded = {col_name[:-len(" Expanded (HL)")]:float(col_val) for col_name, col_val in row.iteritems() \
                                    if col_name in expanded_fieldnames}
            relevant_solvers_and_runtimes = {solver_name:runtime for solver_name, runtime in solvers_and_runtimes.iteritems() \
                                             if solver_name in relevant_solvers_in_category}
            relevant_solvers_and_generated = {solver_name:generated for solver_name, generated in solvers_and_generated.iteritems() \
                                             if solver_name in relevant_solvers_in_category}
            relevant_solvers_and_lookaheads = {solver_name:lookaheads for solver_name, lookaheads in solvers_and_lookaheads.iteritems() \
                                             if solver_name in relevant_solvers_in_category}
            relevant_solvers_and_adoptions = {solver_name:adoptions for solver_name, adoptions in solvers_and_adoptions.iteritems() \
                                             if solver_name in relevant_solvers_in_category}
            relevant_solvers_and_conflicts_solved_with_adoption = {solver_name:conflicts_solved_with_adoption for solver_name, conflicts_solved_with_adoption in solvers_and_conflicts_solved_with_adoption.iteritems() \
                                             if solver_name in relevant_solvers_in_category}
            relevant_solvers_and_nodes_with_goal_cost = {solver_name:nodes_with_goal_cost for solver_name, nodes_with_goal_cost in solvers_and_nodes_with_goal_cost.iteritems() \
                                                         if solver_name in relevant_solvers_in_category}
            relevant_solvers_and_expanded = {solver_name:expanded for solver_name, expanded in solvers_and_expanded.iteritems() \
                                             if solver_name in relevant_solvers_in_category}
            
        except (TypeError, ValueError) as e:
            print "Problem in row: num agents=", row["Num Of Agents"], " instance id=", row["Instance Id"]
            #raise
            continue
        
        if relevant_solvers_that_succeeded != relevant_solvers_in_category:
            #print "Not using runtimes of row: num agents={num_agents}, id={id} - {failures} didn't solve it".format(num_agents=row["Num Of Agents"], id=row["Instance Id"], failures=relevant_solvers_in_category - relevant_solvers_that_succeeded)
            continue # If any relevant algorithm failed, skip the problem to avoid making an unfair comparison.
        num_averaged_problems_per_num_of_agents[num_of_agents] += 1
        solver_relevant_runtimes_per_num_of_agents[num_of_agents].update(relevant_solvers_and_runtimes)
        solver_relevant_generated_per_num_of_agents[num_of_agents].update(relevant_solvers_and_generated)
        solver_relevant_lookaheads_per_num_of_agents[num_of_agents].update(relevant_solvers_and_lookaheads)
        solver_relevant_adoptions_per_num_of_agents[num_of_agents].update(relevant_solvers_and_adoptions)
        solver_relevant_conflicts_solved_with_adoption_per_num_of_agents[num_of_agents].update(relevant_solvers_and_conflicts_solved_with_adoption)
        solver_relevant_nodes_with_goal_cost_per_num_of_agents[num_of_agents].update(relevant_solvers_and_nodes_with_goal_cost)
        solver_relevant_expanded_per_num_of_agents[num_of_agents].update(relevant_solvers_and_expanded)
        solver_relevant_run_count_per_num_of_agents[num_of_agents].update(relevant_solvers_and_runtimes.iterkeys())

# Average the results:
solver_average_runtimes_per_num_of_agents = defaultdict(partial(defaultdict, lambda : None)) # Plot average runtime for solvers that weren't averaged for that category as missing data
for num_of_agents, solver_runtimes in solver_relevant_runtimes_per_num_of_agents.iteritems():
    for solver, runtime in solver_runtimes.iteritems():
        solver_average_runtimes_per_num_of_agents[num_of_agents][solver] = runtime / solver_relevant_run_count_per_num_of_agents[num_of_agents][solver]
        
solver_average_generated_per_num_of_agents = defaultdict(partial(defaultdict, lambda : None)) # Plot average runtime for solvers that weren't averaged for that category as missing data
for num_of_agents, solver_generated in solver_relevant_generated_per_num_of_agents.iteritems():
    for solver, generated in solver_generated.iteritems():
        solver_average_generated_per_num_of_agents[num_of_agents][solver] = generated / solver_relevant_run_count_per_num_of_agents[num_of_agents][solver]
        
solver_average_lookaheads_per_num_of_agents = defaultdict(partial(defaultdict, lambda : None)) # Plot average runtime for solvers that weren't averaged for that category as missing data
for num_of_agents, solver_lookaheads in solver_relevant_lookaheads_per_num_of_agents.iteritems():
    for solver, lookaheads in solver_lookaheads.iteritems():
        solver_average_lookaheads_per_num_of_agents[num_of_agents][solver] = lookaheads / solver_relevant_run_count_per_num_of_agents[num_of_agents][solver]        
        
solver_average_adoptions_per_num_of_agents = defaultdict(partial(defaultdict, lambda : None)) # Plot average runtime for solvers that weren't averaged for that category as missing data
for num_of_agents, solver_adoptions in solver_relevant_adoptions_per_num_of_agents.iteritems():
    for solver, adoptions in solver_adoptions.iteritems():
        solver_average_adoptions_per_num_of_agents[num_of_agents][solver] = adoptions / solver_relevant_run_count_per_num_of_agents[num_of_agents][solver]
        
solver_average_conflits_solved_with_adoption_per_num_of_agents = defaultdict(partial(defaultdict, lambda : None)) # Plot average runtime for solvers that weren't averaged for that category as missing data
for num_of_agents, solver_conflicts_solved_with_adoption in solver_relevant_conflicts_solved_with_adoption_per_num_of_agents.iteritems():
    for solver, conflicts_solved_with_adoption in solver_conflicts_solved_with_adoption.iteritems():
        solver_average_conflits_solved_with_adoption_per_num_of_agents[num_of_agents][solver] = conflicts_solved_with_adoption / solver_relevant_run_count_per_num_of_agents[num_of_agents][solver]

solver_average_nodes_with_goal_cost_per_num_of_agents = defaultdict(partial(defaultdict, lambda : None)) # Plot average runtime for solvers that weren't averaged for that category as missing data
for num_of_agents, solver_nodes_with_goal_cost in solver_relevant_nodes_with_goal_cost_per_num_of_agents.iteritems():
    for solver, nodes_with_goal_cost in solver_nodes_with_goal_cost.iteritems():
        solver_average_nodes_with_goal_cost_per_num_of_agents[num_of_agents][solver] = nodes_with_goal_cost / solver_relevant_run_count_per_num_of_agents[num_of_agents][solver]
        
solver_average_expanded_per_num_of_agents = defaultdict(partial(defaultdict, lambda : None)) # Plot average runtime for solvers that weren't averaged for that category as missing data
for num_of_agents, solver_expanded in solver_relevant_expanded_per_num_of_agents.iteritems():
    for solver, expanded in solver_expanded.iteritems():
        solver_average_expanded_per_num_of_agents[num_of_agents][solver] = expanded / solver_relevant_run_count_per_num_of_agents[num_of_agents][solver]
        
# Print relevant average runtimes per category
print
print
print
for num_of_agents, solver_average_runtimes in sorted(solver_average_runtimes_per_num_of_agents.iteritems()):
    print 'Average runtimes for %d agents' % (num_of_agents, )
    pprint(sorted(solver_average_runtimes.iteritems(), key=lambda x: x[1]))
# TeX table:
print "Average runtimes:"
print '|r|' + '|'.join(('r' for solver in sorted_solver_names)) + '|'
print "Agents & Averaged problems # &" + ' & '.join(sorted_solver_names) + r'\\'
for num_of_agents, solver_average_runtime in sorted(solver_average_runtimes_per_num_of_agents.iteritems()):
    min_val = min(solver_average_runtime.itervalues())
    print "{:>2d} & ".format(num_of_agents) +\
        "{:>3d} & ".format(num_averaged_problems_per_num_of_agents[num_of_agents]) + \
        ' & '.join("{:^15s}".format(("" if solver_average_runtime[solver] != min_val else r"\bf{") + \
                   ("{:,.0f}".format(solver_average_runtime[solver]) if solver_average_runtime[solver] is not None else "N/A") + \
                   ("" if solver_average_runtime[solver] != min_val else r"}"))  \
                   for solver in sorted_solver_names) + \
        r'\\'
print
print
print
for num_of_agents, solver_average_generated in sorted(solver_average_generated_per_num_of_agents.iteritems()):
    print "Average high level generated nodes for %d agents" % (num_of_agents, )
    pprint(sorted(solver_average_generated.iteritems(), key=lambda x:x[1]))
# TeX table:
print "Average generated high level nodes:"
print '|r|' + '|'.join(('r' for solver in sorted_solver_names)) + '|'
print "Agents & " + ' & '.join(sorted_solver_names) + r'\\'
for num_of_agents, solver_average_generated in sorted(solver_average_generated_per_num_of_agents.iteritems()):
    min_val = min(solver_average_generated.itervalues())
    print "{:>2d}".format(num_of_agents) + ' & ' + ' & '.join(
        (("" if solver_average_generated[solver] != min_val else r"\bf{") + \
        ("{:10,}".format(round(solver_average_generated[solver], 2)) if solver_average_generated[solver] is not None else "N/A") + \
        ("" if solver_average_generated[solver] != min_val else r"}")) \
        for solver in sorted_solver_names) + r'\\'
print
print
print
for num_of_agents, solver_average_lookaheads in sorted(solver_average_lookaheads_per_num_of_agents.iteritems()):
    print "Average high level lookahead nodes for %d agents" % (num_of_agents, )
    pprint(sorted(solver_average_lookaheads.iteritems(), key=lambda x:x[1]))
# TeX table:
print "Average invocations of the low level:"
print '|r|' + '|'.join(('r' for solver in sorted_solver_names)) + '|'
print "Agents & " + ' & '.join(sorted_solver_names) + r'\\'
for num_of_agents in sorted_num_of_agents:
    values = [solver_average_generated_per_num_of_agents[num_of_agents][solver] + solver_average_lookaheads_per_num_of_agents[num_of_agents][solver] \
              if (solver_average_generated_per_num_of_agents[num_of_agents][solver] is not None and \
              solver_average_lookaheads_per_num_of_agents[num_of_agents][solver] is not None) else 99999999999999999 \
              for solver in sorted_solver_names]
    min_val = min(values)

    print "{:>2d}".format(num_of_agents) + ' & ' + ' & '.join(
        (("" if value != min_val else r"\bf{") + \
        ("{:,}".format(round(value, 2)) if value != 99999999999999999 else "N/A") + \
        ("" if value != min_val else r"}") \
        for value in values)) + r'\\'
print
print
print
for num_of_agents, solver_average_adoptions in sorted(solver_average_adoptions_per_num_of_agents.iteritems()):
    print "Average high level adoptions for %d agents" % (num_of_agents, )
    pprint(sorted(solver_average_adoptions.iteritems(), key=lambda x:x[1]))
print
print
print
for num_of_agents, solver_average_conflicts_solved_with_adoption in sorted(solver_average_conflits_solved_with_adoption_per_num_of_agents.iteritems()):
    print "Average conflicts solved with adoption for %d agents" % (num_of_agents, )
    pprint(sorted(solver_average_conflicts_solved_with_adoption.iteritems(), key=lambda x:x[1]))
    
# Plot the results
point_styles = itertools.cycle(["D", "o", "*", "x", "H", "s", "v"])
point_styles_big_data = ["D", "o", "s", "v", "*"]
point_styles_big = itertools.cycle(point_styles_big_data)
title = ''
if len(input_paths) == 1: # Use input file name as window title and figure title
    title = os.path.splitext(os.path.basename(sys.argv[1]))[0]

fig = plt.figure(1) # Figure ID 1 - success rate alongside average runtime
# Set window title
fig.canvas.set_window_title(title)
# Set figure title
fig.suptitle(title, size="x-large")
plt.subplot(1, 2, 1) # First of two side-by-side figures
for solver in sorted_solver_names:
    plt.plot(sorted_num_of_agents, np.array([100 * solver_success_rate_per_num_of_agents[num_agents][solver] for num_agents in sorted_num_of_agents]), next(point_styles) + "-", label=solver)
plt.title('Success Rates')
plt.xlabel('Number Of Agents')
plt.ylabel('Success Rate (%)')
plt.yscale('linear')
do_legend = True
if do_legend:
    legend = plt.legend(loc='lower left', fontsize="x-small")
    if legend is not None:
        legend.draggable()
plt.ylim(0, 105) # So that 100 isn't on the top edge of the figure

plt.subplot(1, 2, 2) # Second of two side-by-side figures
for solver in sorted_solver_names:
    plt.plot(sorted_num_of_agents, np.array([solver_average_runtimes_per_num_of_agents[num_agents][solver] for num_agents in sorted_num_of_agents]), next(point_styles) + "-", label=solver)
    # TODO: Add the xerr parameter to show the standard dev?
plt.title('Average Runtimes')
plt.xlabel('Number Of Agents')
plt.ylabel('Average Runtime (ms)')
plt.yscale('linear')
do_legend = True
if do_legend:
    legend = plt.legend(loc='upper left', fontsize="x-small")
    if legend is not None:
        legend.draggable()
plt.ylim(0, 300000) # Hack to leave just enough room at the top of the plot for the legend.
                    # Warning: Turns off autoscaling for the axis

fig = plt.figure(2) # Figure ID 2 - Generated and lookahead
# Set window title
fig.canvas.set_window_title(title)
# Set figure title
fig.suptitle(title, size="x-large")
# Separate graphs for generated and lookaheads, less convenient
# plt.subplot(1, 2, 1) # First of two side-by-side figures
# for solver in sorted_solver_names:
    # plt.plot(sorted_num_of_agents, np.array([solver_average_generated_per_num_of_agents[num_agents][solver] for num_agents in sorted_num_of_agents]), next(point_styles) + "-", label=solver)
# plt.title('Average Generated High Level Nodes')
# plt.xlabel('Number Of Agents')
# plt.ylabel('Average Generated High Level Nodes')
# plt.yscale('log')
# plt.legend(loc='upper left', )
# plt.ylim(0, 300000) # Hack to leave just enough room at the top of the plot for the legend.
                      # Warning: Turns off autoscaling for the axis
# plt.subplot(1, 2, 2) # Second of two side-by-side figures
# for solver in sorted_solver_names:
    # plt.plot(sorted_num_of_agents, np.array([solver_average_lookaheads_per_num_of_agents[num_agents][solver] for num_agents in sorted_num_of_agents]), next(point_styles) + "-", label=solver)
# plt.title('Average High Level Look Ahead Nodes')
# plt.xlabel('Number Of Agents')
# plt.ylabel('Average High Level Look Ahead Nodes')
# plt.yscale('log')
# plt.legend(loc='upper left', )
# plt.ylim(0, 300000) # Hack to leave just enough room at the top of the plot for the legend.
                      # Warning: Turns off autoscaling for the axis

#plt.subplot(1, 2, 1) # First of two side-by-side figures
for solver in sorted_solver_names:
    generated_data = np.array([solver_average_generated_per_num_of_agents[num_agents][solver] for num_agents in sorted_num_of_agents])
    plt.plot(sorted_num_of_agents, generated_data, next(point_styles) + "-", label=solver + " generated nodes")

    lookahead_data = np.array([solver_average_lookaheads_per_num_of_agents[num_agents][solver] for num_agents in sorted_num_of_agents])
    if any(lookahead_data): # Don't plot lookaheads for algorithms that never lookahead.
        plt.plot(sorted_num_of_agents, [l + g if (l is not None and g is not None) else None for l, g in itertools.izip(lookahead_data, generated_data)], next(point_styles) + "-", label=solver + " generated + lookahead nodes")
plt.title('Average Generated and lookahead High Level Nodes')
plt.xlabel('Number Of Agents')
plt.ylabel('Average Number Of High Level Nodes')
plt.yscale('linear')
do_legend = True
if do_legend:
    legend = plt.legend(loc='upper left', fontsize="x-small")
    if legend is not None:
        legend.draggable()

fig = plt.figure(3) # Figure ID 3 - adoptions and conflicts bypassed
plt.subplot(1, 2, 1) # First of two side-by-side figures
for solver in sorted_solver_names:
    adoptions_data = np.array([solver_average_adoptions_per_num_of_agents[num_agents][solver] for num_agents in sorted_num_of_agents])
    if any(adoptions_data): # Don't plot adoptions for algorithms that never adopt.
        plt.plot(sorted_num_of_agents, [a + 1 if a is not None else None for a in adoptions_data], next(point_styles) + "-", label=solver + " adoptions") # + 1 to make log scale work...
plt.title('Average Number Of Adoptions')
plt.xlabel('Number Of Agents')
plt.ylabel('Average Adoptions')
plt.yscale('linear')
do_legend = True
if do_legend:
    legend = plt.legend(loc='upper left', fontsize="x-small")
    if legend is not None:
        legend.draggable()

plt.subplot(1, 2, 2) # Second of two side-by-side figures
for solver in sorted_solver_names:
    conflicts_solved_with_adoption_data = np.array([solver_average_conflits_solved_with_adoption_per_num_of_agents[num_agents][solver] for num_agents in sorted_num_of_agents])
    if any(conflicts_solved_with_adoption_data): # Don't plot adoptions for algorithms that never adopt.
        plt.plot(sorted_num_of_agents, [a + 1 if a is not None else None for a in conflicts_solved_with_adoption_data], next(point_styles) + "-", label=solver + " conflicts solved with adoption")
plt.title('Conflicts Solved With Adoption')
plt.xlabel('Number Of Agents')
plt.ylabel('Average Conflicts Solved')
plt.yscale('linear')
do_legend = True
if do_legend:
    legend = plt.legend(loc='upper left', fontsize="x-small")
    if legend is not None:
        legend.draggable()

fig = plt.figure(4) # Figure ID 4
# Set window title
fig.canvas.set_window_title(title)
# Set figure title
fig.suptitle(title, size="x-large")
plt.subplot(1, 2, 1) # First of two side-by-side figures
for solver in sorted_solver_names:
    expanded_data = np.array([solver_average_expanded_per_num_of_agents[num_agents][solver] for num_agents in sorted_num_of_agents])
    plt.plot(sorted_num_of_agents, expanded_data, next(point_styles) + "-", label=solver + " expanded nodes")

    nodes_with_goal_cost_data = np.array([solver_average_nodes_with_goal_cost_per_num_of_agents[num_agents][solver] for num_agents in sorted_num_of_agents])
    plt.plot(sorted_num_of_agents, nodes_with_goal_cost_data, next(point_styles) + "-", label=solver + " nodes expanded with goal cost")
plt.title('Average Expanded High Level Nodes')
plt.xlabel('Number Of Agents')
plt.ylabel('Average Number Of High Level Nodes')
plt.yscale('linear')
do_legend = True
if do_legend:
    legend = plt.legend(loc='upper left', fontsize="x-small")
    if legend is not None:
        legend.draggable()

fig = plt.figure(5) # Figure ID 5
# Set window title
fig.canvas.set_window_title(title) # UPDATE
# Set figure title
fig.suptitle(title, size="x-large")
#plt.subplot(1, 2, 1) # First of two side-by-side figures

#runtime_last_column_data = np.array([runtime for solver, runtime in solver_average_runtimes_per_num_of_agents[max(solver_average_runtimes_per_num_of_agents)].iteritems()])
#sorted_num_of_agents, np.array([solver_average_runtimes_per_num_of_agents[num_agents][solver] for num_agents in sorted_num_of_agents])

#solvers = ('Tom', 'Dick', 'Harry', 'Slim', 'Jim')
#y_pos = np.arange(len(people))
#performance = 3 + 10 * np.random.rand(len(people))
#plt.barh(y_pos, performance)
    
plt.title('Runtimes for %d agents' % (max(solver_average_runtimes_per_num_of_agents), ))
plt.xlabel('Average runtimes')
plt.ylabel('Average Number Of High Level Nodes')
plt.yscale('linear')
#plt.legend(loc='upper left', fontsize="x-small").draggable()

plt.rc('font', size=22) # Actually affects all figures...
fig = plt.figure(6) # Figure ID 6 - success rate big
point_styles_big = itertools.cycle(point_styles_big_data) # Reset the cycle. Color cycle resets automatically.
# Set window title
fig.canvas.set_window_title(title)
for solver in sorted_solver_names:
    plt.plot(sorted_num_of_agents, np.array([100 * solver_success_rate_per_num_of_agents[num_agents][solver] for num_agents in sorted_num_of_agents]), next(point_styles_big) + "-", label=solver, linewidth=4, markersize=15) # defaults are 1, 6
#plt.title('Success Rates')
plt.xlabel('Number Of Agents', fontsize="x-large", )#fontweight="semibold")
#plt.xlim(xmin=10)
#plt.xlim(3, 72)
#plt.xlim(2.5, 13.5)
#plt.xticks(size="x-large")
plt.ylabel('Success Rate (%)', fontsize="x-large", )#fontweight="semibold")
plt.yscale('linear')
#plt.yticks(size="x-large")]
do_legend = True
if do_legend:
    legend = plt.legend(loc='lower left', shadow=True, fancybox=True, title="Solvers", )#fontsize="x-large")
    if legend is not None:
        legend.draggable()
        legend.get_title().set_fontsize("x-large")
        legend.get_title().set_fontweight("semibold")
plt.ylim(0, 105) # So that 100 isn't on the top edge of the figure

fig = plt.figure(7) # Figure ID 7 - average runtimes big
point_styles_big = itertools.cycle(point_styles_big_data) # Reset the cycle. Color cycle resets automatically.
# Set window title
fig.canvas.set_window_title(title)
for solver in sorted_solver_names:
    plt.plot(sorted_num_of_agents, np.array([(solver_average_runtimes_per_num_of_agents[num_agents][solver]/1000. if solver_average_runtimes_per_num_of_agents[num_agents][solver] is not None else None) for num_agents in sorted_num_of_agents]), next(point_styles_big) + "-", label=solver, linewidth=4, markersize=15)
#plt.title('Average Runtimes', fontsize="x-large")
plt.xlabel('Number Of Agents', fontsize="x-large", )#fontweight="semibold")
#plt.xlim(xmin=10)
#plt.xlim(3, 72)
#plt.xlim(2.5, 13.5)
#plt.xticks(size="x-large")
plt.ylabel('Average Runtime (s)', fontsize="x-large", )#fontweight="semibold")
plt.yscale('linear')
#plt.yticks(size="x-large")
do_legend = False
if do_legend:
    legend = plt.legend(loc='upper left', shadow=True, fancybox=True, title="Solvers", )#fontsize="xx-large") # default fontsize is "large"
    if legend is not None:
        legend.draggable()
        legend.get_title().set_fontsize("x-large")
        legend.get_title().set_fontweight("semibold")
        
# fig = plt.figure(8) # Figure ID 8 - success rate and average runtimes side by side big
# point_styles_big = itertools.cycle(["D", "o", "s", "v"]) # Reset the cycle. Color cycle resets automatically.
# Set window title
# fig.canvas.set_window_title(title)
# plt.subplot(1, 2, 1) # First of two side-by-side figures
# plt.subplot(1, 2, 2) # Second of two side-by-side figures
        
plt.show()
#print "not plotting for now"
