#include <stdio.h>
#include <stdlib.h>

int grades[] = {90, 97, 75, 89, 84, 67, 40};

struct results {
  int min_val;
  int max_val;
  float avg_val;
  int med_val;
  int num_A;
  int num_B;
  int num_C;
  int num_D;
  int num_F;
} ans;

int min(int arr[], int i) {
  int mininc = 0; // where in array is the min
  for (int j = 0; j < i; j++) {
    // if array value @ min >= array value @ j, set j       as mininc (new minimum found)
    if (arr[mininc] >= arr[j]) {
      mininc = j;
    }
  }
  return arr[mininc];
}

int max(int arr[], int i) {
  int maxinc = 0;
  for (int j = 0; j < i; j++) {
    // if array value @ max <= array value @ j, set j       as maxinc (new maximum found)
    if (arr[maxinc] <= arr[j]) {
      maxinc = j;
    }
  }
  return arr[maxinc];
}

float avg(int arr[], int i) {
  float avg = 0;
  //for loop that adds up all of the values of the array
  for (int j = 0; j < i; j++) {
	  avg+=arr[j];
  }
  //divide sum of all grades (avg) by number of grades (i)
  avg/=i;
  return avg;
}

int med(int arr[], int n) {
	int i, j, med, temp;
	//use bubble sort to organize grades, then find median
	 for (i = 0; i < n - 1; i++) {
	        for (j = 0; j < n - i - 1; j++) {
	            if (arr[j] > arr[j + 1]) {
	                temp = arr[j];
	                arr[j] = arr[j+1];
	                arr[j+1] = temp;
	            }
	        }
	 }
	 //go to midpoint in array and find value that is there
	 med = arr[n/2];
	 return med;
}

void lettergrader(int arr[], int i){
	//for loop that takes each grade, and determines its letter grade
	for (int j=0;j<i;j++) {
		if (arr[j]>=90) {
			ans.num_A++;
		} else if (arr[j]>=80) {
			ans.num_B++;
		} else if (arr[j]>=70) {
			ans.num_C++;
		} else if (arr[j]>=60) {
			ans.num_D++;
		} else {
			ans.num_F++;
		}
	}
}
int main() {
  int i;
  i = sizeof(grades)/sizeof(int); // to utilize as increment for for loops
  ans.min_val = min(grades, i);
  ans.max_val = max(grades, i);
  ans.avg_val = avg(grades, i);
  ans.med_val = med(grades, i);
  lettergrader(grades,i);


  return 0;
}
