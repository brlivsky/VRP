What is the problem?
Assign vehicles of 2 types to cater to all PD
1. without violating the 
    - capacity constraint
    - vehicle constraint
    - service length / duration
2. minimizing the 
    - cost

Pseudocode to add two types of vehicles
1. choose pd pair having the maximum round trip distance/earliest deadline time
2. add it as the first route and choose vehicle type based on the least fixed cost per capacity of vehicle
3. while there is no remaining unrouted PD pairs
    1. for each of the unrouted PD pairs
        1. add each pd pairs to the existing routes if feasible
        2. create new route if not feasible
        3. calculate the fixed+running cost 
        4. save least cost and corresponding pd pair
        5. remove the added pd
    2. choose that PD pair having least insertion cost 
    3. add that pd pair to the solution
    4. remove PD from set
    

Modifications to be made in
1. Dataset [done]
 - Add 4 values in row 1
    - No of type B vehicles
    - Capacity of type B vehicles
    - Fixed Cost of A 
    - Fixed Cost of B
2. Read dataset function [done]
 - Read the new capacity and no of type B vehicles 
3. Sol must be modified to identify dif vehicle types in the routes [done]
    struct {
        vector<int> typeOfVehicle = {0, 1, 1, 0, 1}; 
        // if no of routes = 5
        // 0 - typeA, 1 - type B
    }
4. Modify solution cost by updating it with fixed + running cost [done]

Note
1. The solution requires atleast one PD pair to add further PD pairs because the alogirthm adds and removes PD pairs to exisitng solution to check the feasibility and cost and choose those solutions which have least cost of insertion. 
2. The service length limit should be always >= the maximum of 0 - P - D - 0; where P is an element of pikcup set and D an element of delivery set. 

Criterias of choosing vehicle
1. vehicle availability: if one vehicle type is available and other type is not 
   -> assign vehicle that is avaibale
2. least cost (cost/capacity): if both vehicle types are available 
   -> assign vehicle with least cost/capacity

