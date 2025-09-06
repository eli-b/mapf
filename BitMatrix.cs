using System.Collections;

namespace mapf;

/// <summary>
/// BitArray-based matrix for storing boolean values.
/// </summary>
public class BitMatrix
{
    private readonly BitArray[] _bitArrays;

    public BitMatrix(int cols, int rows)
    {
        _bitArrays = new BitArray[cols];
        for (int i = 0; i < cols; i++)
        {
            _bitArrays[i] = new BitArray(rows);
        }
    }

    public int RowsCount => _bitArrays[0].Length;

    public int ColumnsCount => _bitArrays.Length;

    public void Set(int col, int row, bool value) => _bitArrays[col][row] = value;

    public bool Get(int col, int row) => _bitArrays[col][row];

    public bool this[int col, int row]
    {
        get => Get(col, row);
        set => Set(col, row, value);
    }
}
