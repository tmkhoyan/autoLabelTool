# autoLabelTool
A simple but effective automatic annotation tool with a flexible customisable detector for pixel level annotation accuracy 

## Installation 

### 1: go to mex folder

```bash
cd mex/
```


## 4 : References
Please use the following DOI to cite cvyamlParser: 
[![DOI](https://zenodo.org/badge/184505001.svg)](https://zenodo.org/badge/latestdoi/184505001)

## 5: Licence 
Please refer to the licence file for information about code distribution, usage and copy rights. The code is provided under BSD 3-Clause License. 
Refer to the links below for additional licence info regarding OpenCV and Matlab:
https://opencv.org/license/
https://in.mathworks.com/pricing-licensing.html

## Usage
call readcvYaml on the dataset of choice. The function takes as input the filename and the sort option. By default readcvYaml will parse the variables names listed in the yaml file and assign this to a structure with corresponding fields. E.g.:

```Matlab
s = readcvYaml('../data/test_data.yaml')
s = 
  struct with fields:

       matA0: [1000×3 double]
       matA1: [1000×3 double]
       matA2: [1000×3 double]
```
In readcvYaml a handy option is implemented to sort the data based on basename and numeric identifier. When using the sorting option entries that have a unique basename will be folded into multidimentional struct. This is very handy when you have similar datasets that belong to the same category or experimental condition etc. This is done like so:
```Matlab
s = readcvYaml('../data/test_data.yaml','sorted')
s = 
  struct with fields:

        matA: [1×3 struct]
```
The sorting then stores the matrices with matA basename in 2d strructure that can be accessed with:
```Matlab
s.matA(1).matA
```
The numerical identifier does not have to be continuous, the sorting wil sort and store in ascending order. I.e.: A1, A2, A7, A12 and so forth. s.matA(1).index stores the numerical identifier.

The parser will automatically identify the datatype of the stored variable and return this in the structure. It is able to handle all common types used in OpenCV and Matlab environments. Common datatypes are that are returned from OpenCv to matlab: 

```C++
OpenCV 		--> 	Matlab  	--	sizeof 
CV_8U ,CV_8US 	-->	int8_t(char)	--	1
CV_16S,CV_16U	-->	short		--	2
CV_32S		-->	int		--	4
CV_32F		-->	float		--	4
CV_64F		-->	double		--	8
```
The parser can convert vectors, matrices and single variables stored in yaml file. Although untested it should also work with xml files.
Refer to the test_data.yaml and genyamlData.cpp see an example of how the data is generated.

## Benchmarking

<!---
### 2: Run benchmark on you own pc:
In folder benchmark a simple script is provided to run readcvYaml on your own data.
Simply choose the number of iterations with N parameter and run benchmarktest_cvYaml.m. The benchmark was performed for 5x[1000x3] double, 5x[2000x3] float, 5x[2000x3] int, and 5x[3000] double, 5x[6000] float and 5x[6000] int vectors. See test_data.yaml for the actual dataset.

Here the result of the benchmark test on linux Optiplex system. The sorting is slightly more expensive as expected but negligible for the current dataset.

Average t per iteration    |  Boxplot data
:-------------------------:|:-------------------------:
![](misc/time_data.png)    |  ![](misc/boxplot.png)

-->
