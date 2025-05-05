using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace mapf;

public interface IHeuristicSearchNode
{
    int G { get; set; }
    int H { get; set; }
    /// <summary>
    /// Used to mark that heuristic estimate was improved already
    /// </summary>
    int HBonus { get; set; }
    int F { get; }

    bool GoalTest();

    /// <summary>
    /// Returns the h needed to set (raise) the node's f to the given value
    /// </summary>
    /// <param name="f"></param>
    /// <returns></returns>
    int GetTargetH(int f);

    // TODO: Consider changing all the ints above to uints
}
