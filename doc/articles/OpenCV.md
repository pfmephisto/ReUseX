# OpenCV

[here](https://stackoverflow.com/questions/10167534/how-to-find-out-what-type-of-a-mat-object-is-with-mattype-in-opencv#39780825)
For debugging purposes in case you want to look up a raw Mat::type in a debugger:

|            	| **C1** 	| **C2** 	| **C3** 	| **C4** 	| **C(5)** 	| **C(6)** 	| **C(7)** 	| **C(8)** 	|
|------------	|:------:	|:------:	|:------:	|:------:	|:--------:	|:--------:	|:--------:	|:--------:	|
| **CV_8U**  	|    0   	|    8   	|   16   	|   24   	|    32    	|    40    	|    48    	|    56    	|
| **CV_8S**  	|    1   	|    9   	|   17   	|   25   	|    33    	|    41    	|    49    	|    57    	|
| **CV_16U** 	|    2   	|   10   	|   18   	|   26   	|    34    	|    42    	|    50    	|    58    	|
| **CV_16S** 	|    3   	|   11   	|   19   	|   27   	|    35    	|    43    	|    51    	|    59    	|
| **CV_32S** 	|    4   	|   12   	|   20   	|   28   	|    36    	|    44    	|    52    	|    60    	|
| **CV_32F** 	|    5   	|   13   	|   21   	|   29   	|    37    	|    45    	|    53    	|    61    	|
| **CV_64F** 	|    6   	|   14   	|   22   	|   30   	|    38    	|    46    	|    54    	|    62    	|


So for example, if type = 30 then OpenCV data type is `CV_64FC4`. If type = 50 then the OpenCV data type is `CV_16UC(7)`.

