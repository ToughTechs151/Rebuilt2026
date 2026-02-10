package frc.robot.util;

import java.util.Arrays;

public class LUT {
  double[][] table;

  /**
   * Create a look up table using a nested list <br>
   * Assumes key values are in increasing order, and does not sort them for you. <br>
   * Assumes 2 or more rows, and an equal number of columns. <br>
   * Example:
   *
   * <pre lang="java">
   * var lut = LUT(new double[][]{{
   * {0,1,2},
   * {1,2,4},
   * {3,6,36}
   * });
   * lut.get(2); // would yield {2,4,20}
   * </pre>
   *
   * @param table of interpolated values
   */
  public LUT(double[][] table) {
    this.table = table;
  }

  /**
   * @param key
   * @return row of interpolated values {key,val1,val2....}
   */
  public double[] get(double key) {
    // if we're smaller than our smallest key, return smallest value
    if (key < table[0][0]) {
      return Arrays.copyOfRange(table[0], 0, table[0].length);
    }

    // if we're larger than our largest value, return largest value
    if (key > table[table.length - 1][0]) {
      return Arrays.copyOfRange(table[table.length - 1], 0, table[table.length - 1].length);
    }

    // with those out of the way, we can safely assume we're in between two points that exist.
    // Find it, split the difference
    for (var row = 0; row < table.length - 1; row++) {

      // Check to see if the _next_ row is larger than our key value.
      // This implies the current row value is the one below it.
      if (table[row + 1][0] > key) {
        // Interpolate between this row and the next, and return those values
        double[] value = new double[table[row].length];
        value[0] = key;
        for (var col = 1; col < table[row].length; col++) {
          value[col] =
              Lerp.lerp(
                  key, table[row][0], table[row + 1][0], table[row][col], table[row + 1][col]);
        }
        return value;
      }
    }

    // This should be unreachable, but returns the last list item
    return Arrays.copyOfRange(table[table.length - 1], 0, table[table.length - 1].length);
  }
}
