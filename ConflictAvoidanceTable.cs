using System.Collections.Generic;
using System.Linq;

namespace mapf;

public enum AvoidanceGoal
{
    MINIMIZE_CONFLICTS,
    MINIMIZE_CONFLICTING_GROUPS_THEN_CONFLICTS,
    MINIMIZE_CONFLICTING_GROUPS,
    MINIMIZE_LARGEST_CONFLICTING_GROUP_THEN_NUMBER_OF_SUCH_GROUPS,
    MINIMIZE_LARGEST_CONFLICTING_GROUP_THEN_MAXIMIZE_CONFLICT_COUNTS_WITH_OTHERS,
    MINIMIZE_CONFLICTING_GROUP_SIZE_AND_COUNT
};

public class ConflictAvoidanceTable
{
    private Dictionary<TimedMove, List<int>> _timedMovesToAgentNumList = [];
    private Dictionary<Move, (int time, int agentNum)> _atGoalWaitsToTimeAndAgentNum = []; // No need for a list of agent nums because goals can't collide :)

    public Dictionary<int, int> AgentSizes { get; } = [];
    public Dictionary<int, int> AgentConflictCounts { get; } = [];

    public int NumPlans { get; private set; } = 0;

    public AvoidanceGoal AvoidanceGoal { get; set; } = AvoidanceGoal.MINIMIZE_CONFLICTING_GROUPS_THEN_CONFLICTS;  // Better for CBS

    public ConflictAvoidanceTable()
    {
    }

    public int GetMaxPlanSize()
    {
        if (_atGoalWaitsToTimeAndAgentNum.Count > 0)
            return _atGoalWaitsToTimeAndAgentNum.Values.Max(tuple => tuple.time) - 1;  // The first WAIT at goal we record is one time step after reaching it
        else
            return 0;
    }

    public void Clear()
    {
        _timedMovesToAgentNumList.Clear();
        _atGoalWaitsToTimeAndAgentNum.Clear();
        AgentSizes.Clear();
        AgentConflictCounts.Clear();
        NumPlans = 0;
    }
        
    public void AddPlan(SinglePlan plan)
    {
        int planSize = plan.GetSize();
        for (int i = 0; i < planSize; i++)
        {
            Move temp = plan.GetLocationAt(i);
            TimedMove step;
            if (temp.GetType() == typeof(TimedMove))
                step = (TimedMove)temp;
            else  // It's a Move object
            {
                step = new TimedMove(temp, i);
            }
            if (_timedMovesToAgentNumList.ContainsKey(step) == false)
            {
                _timedMovesToAgentNumList[step] = [ plan.AgentNum ];
            }
            else
                _timedMovesToAgentNumList[step].Add(plan.AgentNum);
        }

        Move lastMove = plan.GetLocationAt(planSize - 1);
        Move goal = new(lastMove.X, lastMove.Y, Direction.Wait);
        _atGoalWaitsToTimeAndAgentNum[goal] = (planSize, plan.AgentNum);
        ++NumPlans;
    }

    public void RemovePlan(SinglePlan plan)
    {
        int planSize = plan.GetSize();
        for (int i = 0; i < planSize; i++)
        {
            Move temp = plan.GetLocationAt(i);
            TimedMove step;
            if (temp.GetType() == typeof(TimedMove))
                step = (TimedMove)temp;
            else  // It's a Move object
            {
                step = new TimedMove(temp, i);
            }
            _timedMovesToAgentNumList[step].Remove(plan.AgentNum);
            // TODO: Add asserts that check the plan was indeed in the CAT
        }

        Move lastMove = plan.GetLocationAt(planSize - 1);
        Move indexMove = new(lastMove.X, lastMove.Y, Direction.Wait);
        _atGoalWaitsToTimeAndAgentNum.Remove(indexMove);
        --NumPlans;
    }

    /// <summary>
    /// Gets the element that has the specified key in the read-only dictionary.
    /// </summary>
    /// <param name="key">The key to locate</param>
    /// <returns>The element that has the specified key in the read-only dictionary</returns>
    /// <exception cref="System.ArgumentNullException">key is null</exception>
    /// <exception cref="System.Collections.Generic.KeyNotFoundException">The property is retrieved and key is not found</exception>
    public IReadOnlyList<int> this[TimedMove key]
    {
        get
        {
            List<int> ans = null;
            if (_timedMovesToAgentNumList.TryGetValue(key, out List<int> value))
            {
                ans = new List<int>(value.Count + 1);
                ans.AddRange(value);
            }
                
            Move indexMove = new(key.X, key.Y, key.Direction);
            if (_atGoalWaitsToTimeAndAgentNum.TryGetValue(indexMove, out (int time, int agentNum) timeAndAgentNum))
            {
                if (key.Time >= timeAndAgentNum.time)
                {
                    if (ans == null)
                        ans = [ timeAndAgentNum.agentNum ];
                    else
                        ans.Add(timeAndAgentNum.agentNum);
                }
            }

            if (ans != null)
                return ans;
            else
                return [];
        }
    }

    /// <summary>
    /// Determines whether the read-only dictionary contains an element that has
    ///  the specified key.
    /// </summary>
    /// <param name="key">The key to locate.</param>
    /// <returns>true if the read-only dictionary contains an element that has the specified key;
    /// otherwise, false.</returns>
    /// <exception cref="System.ArgumentNullException">key is null</exception>
    public bool ContainsKey(TimedMove key)
    {
        if (_timedMovesToAgentNumList.ContainsKey(key))
            return true;

        Move indexMove = new(key.X, key.Y, key.Direction);
        if (_atGoalWaitsToTimeAndAgentNum.TryGetValue(indexMove, out (int time, int agentNum) value))
        {
            if (key.Time >= value.time)
                return true;
        }
        return false;
    }
}
