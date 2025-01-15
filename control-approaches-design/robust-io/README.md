# How to setup
## Matlab
First install Matlab, we tested with version R2023b Update 4 on Windows 10.  
Then install the Yalmip and Mosek packages.  

## Yalmip
Download https://github.com/yalmip/yalmip/archive/master.zip  
Unzip in folder "yalmip" (C:\Program Files\MATLAB\yalmip)  
Navigate inside Matlab to folder (cd C:\Program Files\MATLAB\yalmip)  
Add to Matlab path with: addpath(genpath(pwd))  
Save path with: savepath
Test with: yalmiptest

## Mosek
Get personal academic license at https://www.mosek.com/products/academic-licenses/  
Place license under C:\Users\USERNAME\mosek\mosek.lic  
Download installer from https://www.mosek.com/downloads/  
Install in folder "mosek" (C:\Program Files\MATLAB\mosek)  
Reboot.  
Navigate to folder mosek  
Add to Matlab path with: addpath(genpath(pwd))  
Save path with: savepath  
Test with: yalmiptest ENTER ENTER  

# How to use
Open and run the RobustIO.m file 

The 5 output values need to be copied into the receiver config file like this:  
The first value of K_iqc is the integral feedback (Kxic), called 'controllerIntegratorParam' in the config file.  
The following 4 values are Kxc, Kvc, Kxp, Kvp and used in this order for 'controllerKVector'.

If you receive an error about gamma_iqc = NaN, there is likely a problem with the mosek license.
Run yalmiptest and press ENTER twice to test mosek. If you recently updated the license yet it still showes "Failed" for the mosek tests, restart Matlab and try again.
