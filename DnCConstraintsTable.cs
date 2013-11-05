//using System;
//using System.Collections.Generic;
//using System.Collections;
//using System.Linq;
//using System.Text;

//namespace CPF_experement
//{
//    class DnCConstraintsTable
//    {
//        DynamicArray<LinkedList<Coordinate>>[] allConstraints; //agent number -> timeStep -> locations banded
//        public int count;
//        int hashKey;

//        public DnCConstraintsTable(int agentsCount)
//        {
//            allConstraints = new DynamicArray<LinkedList<Coordinate>>[agentsCount];
//            for (int i = 0; i < agentsCount; i++)
//            {
//                allConstraints[i] = new DynamicArray<LinkedList<Coordinate>>();
//            }
//            this.count = 0;
//            this.hashKey = -1;
//        }

//        public DnCConstraintsTable(DnCConstraintsTable cpy)
//        {
//            this.allConstraints = new DynamicArray<LinkedList<Coordinate>>[cpy.allConstraints.Length];
//            for (int i = 0; i < cpy.allConstraints.Length; i++)
//            {
//                this.allConstraints[i] = new DynamicArray<LinkedList<Coordinate>>();
//                for (int j = 0; j < cpy.allConstraints[i].getLength(); j++)
//                {
//                    if (cpy.allConstraints[i][j] != null)
//                    {
//                        this.allConstraints[i][j] = new LinkedList<Coordinate>();
//                        foreach (Coordinate cord in cpy.allConstraints[i][j])
//                        {
//                            this.allConstraints[i][j].AddFirst(cord);
//                        }
//                    }
//                }
//            }
//            this.count = cpy.count;
//            this.hashKey = -1;
//        }

//        public void Add(DnCConstraint constraint)
//        {
//            Coordinate illegal = new Coordinate(constraint.getX(), constraint.getY(),constraint.getDirection());
//            int agent = constraint.getAgentNum();
//            int timeStep = constraint.getTimeStep();
//            if (allConstraints[agent][timeStep] == null)
//                allConstraints[agent][timeStep] = new LinkedList<Coordinate>();
//            allConstraints[agent][timeStep].AddFirst(illegal);
//            count++;
//        }

//        public bool Contains(DnCConstraint constraint)
//        {
//            Coordinate illegal = new Coordinate(constraint.getX(), constraint.getY(), constraint.getDirection());
//            int agent = constraint.getAgentNum();
//            int timeStep = constraint.getTimeStep();
//            if (allConstraints[agent][timeStep] == null)
//                return false;
//            return allConstraints[agent][timeStep].Contains(illegal);
//        }

//        public override int GetHashCode()
//        {
//            if (this.hashKey > -1)
//                return this.hashKey;
//            int ans = 0;
//            for (int i = 0; i < this.allConstraints.Length; i++)
//            {
//                for (int j = 0; j < this.allConstraints[i].getLength(); j++)
//                {
//                    if (this.allConstraints[i][j] != null)
//                    {
//                        foreach (Coordinate cord in this.allConstraints[i][j])
//                        {
//                            ans += (i + 1) * (j + 1) * (cord.getX() + 1) * (cord.getY() + 1);
//                        }
//                    }
//                }
//            }
//            this.hashKey = ans;
//            return ans;
//        }

//        public override bool Equals(object obj)
//        {
//            DnCConstraintsTable other = (DnCConstraintsTable)obj;
//            if (this.count != other.count)
//                return false;
//            for (int i = 0; i < this.allConstraints.Length; i++)
//            {
//                if (this.allConstraints[i].getCount() != other.allConstraints[i].getCount())
//                    return false;
//                for (int j = 0; j < this.allConstraints[i].getLength(); j++)
//                {
//                    if (this.allConstraints[i][j] != null)
//                    {
//                        foreach (Coordinate cord in this.allConstraints[i][j])
//                        {
//                            if (other.allConstraints[i][j] == null || other.allConstraints[i][j].Contains(cord) == false)
//                                return false;
//                        }
//                    }
//                }
//            }
//            return true;
//        }
//    }
//}
