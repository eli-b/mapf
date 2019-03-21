using System;
using System.IO;
using System.Collections.Generic;

namespace CPF_experiment
{
    class RandomChoiceOfHeuristic<State> : IHeuristicCalculator<State>
    {
        protected IHeuristicCalculator<State> first;
        protected IHeuristicCalculator<State> second;
        protected double p;
        protected Random rand;

        /// <summary>
        /// 
        /// </summary>
        /// <param name="first"></param>
        /// <param name="second"></param>
        /// <param name="p"></param>
        /// <param name="seed"></param>
        public RandomChoiceOfHeuristic(IHeuristicCalculator<State> first,
            IHeuristicCalculator<State> second, double p, int seed = 0)
        {
            this.first = first;
            this.second = second;
            this.p = p;
            this.rand = new Random(seed);
        }

        public override string ToString()
        {
            return "RandomChoiceOfHeuristic(" + this.p + ":" + this.first + " " + (1 - this.p) + ":" + this.second + ")";
        }

        public string GetName()
        {
            return this.ToString();
        }

        public uint h(State s)
        {
            if (this.rand.NextDouble() < p)
                return this.first.h(s);
            else
                return this.second.h(s);
        }

        public void Init(ProblemInstance pi, List<uint> agentsToConsider)
        {
            this.first.Init(pi, agentsToConsider);
            this.second.Init(pi, agentsToConsider);
        }

        public virtual void OutputStatisticsHeader(TextWriter output)
        {
            this.first.OutputStatisticsHeader(output);
            this.second.OutputStatisticsHeader(output);
        }

        public virtual void OutputStatistics(TextWriter output)
        {
            this.first.OutputStatistics(output);
            this.second.OutputStatistics(output);
        }

        public virtual int NumStatsColumns
        {
            get
            {
                return this.first.NumStatsColumns + this.second.NumStatsColumns;
            }
        }

        public virtual void ClearStatistics()
        {
            this.first.ClearStatistics();
            this.second.ClearStatistics();
        }

        public virtual void ClearAccumulatedStatistics()
        {
            this.first.ClearAccumulatedStatistics();
            this.second.ClearAccumulatedStatistics();
        }

        public virtual void AccumulateStatistics()
        {
            this.first.AccumulateStatistics();
            this.second.AccumulateStatistics();
        }

        public virtual void OutputAccumulatedStatistics(TextWriter output)
        {
            this.first.OutputAccumulatedStatistics(output);
            this.second.OutputAccumulatedStatistics(output);
        }
    }
}
