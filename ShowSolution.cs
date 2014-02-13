using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace CPF_experiment
{
    public partial class ShowSolution : Form
    {
        public ShowSolution(ProblemInstance ins,Plan solution)
        {
            InitializeComponent();
            pictureBox1.Show();
            dataGridView1.ColumnCount = ins.GetMaxX();
            dataGridView1.RowCount = ins.GetMaxY();
            bool[][] grid = ins.m_vGrid;
            for (int i = 0; i < grid.Length; i++)
            {
                for (int j = 0; j < grid[i].Length; j++)
                {
                    if (grid[i][j])
                        dataGridView1[i, j].Style.BackColor = Color.Black;
                }
            }
            dataGridView1.Show();
        }
    }
}
