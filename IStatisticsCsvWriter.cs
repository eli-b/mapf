using System;
using System.IO;

namespace mapf
{
    public interface IStatisticsCsvWriter
    {
        /// <summary>
        /// Prints header of statistics of a single run to the given output. 
        /// </summary>
        void OutputStatisticsHeader(TextWriter output);

        /// <summary>
        /// Prints statistics of a single run to the given output.
        /// </summary>
        void OutputStatistics(TextWriter output);

        /// <summary>
        /// To fill out them out when an algorithm isn't run
        /// </summary>
        int NumStatsColumns { get; }

        /// <summary>
        /// Clears statistics.
        /// </summary>
        void ClearStatistics();
    }

    public interface IAccumulatingStatisticsCsvWriter : IStatisticsCsvWriter
    {
        void ClearAccumulatedStatistics();
        void AccumulateStatistics();
        void OutputAccumulatedStatistics(TextWriter output);
    }
}
