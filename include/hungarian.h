// Copyright 2010-2024 Google LLC
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

namespace mono_lane_mapping {

class HungarianOptimizer {
  static constexpr int kHungarianOptimizerRowNotFound = -1;
  static constexpr int kHungarianOptimizerColNotFound = -2;

 public:
  // Setup the initial conditions for the algorithm.

  // Parameters: costs is a matrix of the cost of assigning each agent to
  // each task. costs[i][j] is the cost of assigning agent i to task j.
  // All the costs must be non-negative.  This matrix does not have to
  // be square (i.e. we can have different numbers of agents and tasks), but it
  // must be regular (i.e. there must be the same number of entries in each row
  // of the matrix).
  explicit HungarianOptimizer(const std::vector<std::vector<double>> &costs);

  // Find an assignment which maximizes the total cost.
  // Returns the assignment in the two vectors passed as argument.
  // preimage[i] is assigned to image[i].
  void Maximize(std::vector<int> *preimage, std::vector<int> *image);

  // Like Maximize(), but minimizing the cost instead.
  void Minimize(std::vector<int> *preimage, std::vector<int> *image);

 private:
  typedef void (HungarianOptimizer::*Step)();

  typedef enum { NONE, PRIME, STAR } Mark;

  // Convert the final cost matrix into a set of assignments of preimage->image.
  // Returns the assignment in the two vectors passed as argument, the same as
  // Minimize and Maximize
  void FindAssignments(std::vector<int> *preimage, std::vector<int> *image);

  // Is the cell (row, col) starred?
  bool IsStarred(int row, int col) const { return marks_[row][col] == STAR; }

  // Mark cell (row, col) with a star
  void Star(int row, int col) {
    marks_[row][col] = STAR;
    stars_in_col_[col]++;
  }

  // Remove a star from cell (row, col)
  void UnStar(int row, int col) {
    marks_[row][col] = NONE;
    stars_in_col_[col]--;
  }

  // Find a column in row 'row' containing a star, or return
  // kHungarianOptimizerColNotFound if no such column exists.
  int FindStarInRow(int row) const;

  // Find a row in column 'col' containing a star, or return
  // kHungarianOptimizerRowNotFound if no such row exists.
  int FindStarInCol(int col) const;

  // Is cell (row, col) marked with a prime?
  bool IsPrimed(int row, int col) const { return marks_[row][col] == PRIME; }

  // Mark cell (row, col) with a prime.
  void Prime(int row, int col) { marks_[row][col] = PRIME; }

  // Find a column in row containing a prime, or return
  // kHungarianOptimizerColNotFound if no such column exists.
  int FindPrimeInRow(int row) const;

  // Remove the prime marks_ from every cell in the matrix.
  void ClearPrimes();

  // Does column col contain a star?
  bool ColContainsStar(int col) const { return stars_in_col_[col] > 0; }

  // Is row 'row' covered?
  bool RowCovered(int row) const { return rows_covered_[row]; }

  // Cover row 'row'.
  void CoverRow(int row) { rows_covered_[row] = true; }

  // Uncover row 'row'.
  void UncoverRow(int row) { rows_covered_[row] = false; }

  // Is column col covered?
  bool ColCovered(int col) const { return cols_covered_[col]; }

  // Cover column col.
  void CoverCol(int col) { cols_covered_[col] = true; }

  // Uncover column col.
  void UncoverCol(int col) { cols_covered_[col] = false; }

  // Uncover ever row and column in the matrix.
  void ClearCovers();

  // Find the smallest uncovered cell in the matrix.
  double FindSmallestUncovered() const;

  // Find an uncovered zero and store its coordinates in (zeroRow_, zeroCol_)
  // and return true, or return false if no such cell exists.
  bool FindZero(int *zero_row, int *zero_col) const;

  // Print the matrix to stdout (for debugging.)
  void PrintMatrix();

  // Run the Munkres algorithm!
  void DoMunkres();

  // Step 1.
  // For each row of the matrix, find the smallest element and subtract it
  // from every element in its row.  Go to Step 2.
  void ReduceRows();

  // Step 2.
  // Find a zero (Z) in the matrix.  If there is no starred zero in its row
  // or column, star Z.  Repeat for every element in the matrix.  Go to step 3.
  // Note: profiling shows this method to use 9.2% of the CPU - the next
  // slowest step takes 0.6%.  I can't think of a way of speeding it up though.
  void StarZeroes();

  // Step 3.
  // Cover each column containing a starred zero.  If all columns are
  // covered, the starred zeros describe a complete set of unique assignments.
  // In this case, terminate the algorithm.  Otherwise, go to step 4.
  void CoverStarredZeroes();

  // Step 4.
  // Find a noncovered zero and prime it.  If there is no starred zero in the
  // row containing this primed zero, Go to Step 5.  Otherwise, cover this row
  // and uncover the column containing the starred zero. Continue in this manner
  // until there are no uncovered zeros left, then go to Step 6.
  void PrimeZeroes();

  // Step 5.
  // Construct a series of alternating primed and starred zeros as follows.
  // Let Z0 represent the uncovered primed zero found in Step 4.  Let Z1 denote
  // the starred zero in the column of Z0 (if any). Let Z2 denote the primed
  // zero in the row of Z1 (there will always be one).  Continue until the
  // series terminates at a primed zero that has no starred zero in its column.
  // Unstar each starred zero of the series, star each primed zero of the
  // series, erase all primes and uncover every line in the matrix.  Return to
  // Step 3.
  void MakeAugmentingPath();

  // Step 6.
  // Add the smallest uncovered value in the matrix to every element of each
  // covered row, and subtract it from every element of each uncovered column.
  // Return to Step 4 without altering any stars, primes, or covered lines.
  void AugmentPath();

  // The size of the problem, i.e. max(#agents, #tasks).
  int matrix_size_;

  // The expanded cost matrix.
  std::vector<std::vector<double>> costs_;

  // The greatest cost in the initial cost matrix.
  double max_cost_;

  // Which rows and columns are currently covered.
  std::vector<bool> rows_covered_;
  std::vector<bool> cols_covered_;

  // The marks_ (star/prime/none) on each element of the cost matrix.
  std::vector<std::vector<Mark>> marks_;

  // The number of stars in each column - used to speed up coverStarredZeroes.
  std::vector<int> stars_in_col_;

  // Representation of a path_ through the matrix - used in step 5.
  std::vector<int> preimage_;  // i.e. the agents
  std::vector<int> image_;     // i.e. the tasks

  // The width_ and height_ of the initial (non-expanded) cost matrix.
  int width_;
  int height_;

  // The current state of the algorithm
  HungarianOptimizer::Step state_;
};

}  // namespace mono_lane_mapping
