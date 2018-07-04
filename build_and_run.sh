#!/bin/bash                  

# update this repo                                                                                                                                                                                                                   
git pull                         
          
#make binary		  
pushd sdk                                                                                                               
make                                                                                                                    
popd                                    
            
#run it			
./sdk/output/Linux/Release/ultra_simple 
