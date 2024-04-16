/*
Reads a single text file.
PD pairs are sorted based on largest round trip distance and earliest deadline
Shift operator
Exchange operator
*/

#include <iostream>
#include <unordered_map>
#include <iomanip>
#include <climits>
#include <bits/stdc++.h>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <string.h>
#include <cmath>

using namespace std;

struct Sol
{
    int NV;
    float TC;
    float SD;
    float WT;
    vector<vector<int>> vectRoute; // n (vehciles/routes) x m (no of locations in each route)
    bool operator<(const Sol &other) const
    {
        if (TC != other.TC)
            return TC < other.TC;
        if (SD != other.SD)
            return SD < other.SD;
        if (WT != other.WT)
            return WT < other.WT;
        return NV < other.NV;
    }
};

template <typename S>
auto select_random(const S &s, size_t n)
{
    auto it = std::begin(s);
    std::advance(it, n);
    return it;
}

void printVector(vector<int> a)
{
    for (auto x : a)
        std::cout << x << " ";
    std::cout << endl;
}

float dist(vector<int> vTask1, vector<int> vTask2)
{
    float fVal = sqrt(pow(vTask1.at(0) - vTask2.at(0), 2) + pow(vTask1.at(1) - vTask2.at(1), 2));
    return floor(fVal * 100.0) / 100.0;
}

Sol populateSolution(Sol SolX, unordered_map<int, vector<int>> mapTask)
{
    int NV = SolX.vectRoute.size();
    float TC = 0, SD = 0, WT = 0;
    vector<float> vectSD(SolX.vectRoute.size(), 0);
    int nStartTimeIdx = 3;
    int nEndTimeIdx = 4;
    int nServiceTimeIdx = 5;
    int nPickupIdx = 6;
    int nDeliveryIdx = 7;
    for (int i = 0; i < SolX.vectRoute.size(); i++)
    {
        for (int j = 0; j < SolX.vectRoute.at(i).size(); j++)
        {
            int nCurTask = SolX.vectRoute[i][j];
            if (j == 0 || j == SolX.vectRoute.at(i).size() - 1)
            {
                TC += dist(mapTask.at(nCurTask), mapTask.at(0));
                vectSD[i] += dist(mapTask.at(nCurTask), mapTask.at(0));
            }
            if (j == 0)
            {
                if (vectSD[i] < mapTask.at(nCurTask).at(nStartTimeIdx))
                {
                    WT += mapTask.at(nCurTask).at(nStartTimeIdx) - vectSD[i];
                    // if (WT == 0)
                    // {
                    //     std::cout << "WT is 0\n";
                    //     std::this_thread::sleep_for(50ms);
                    // }
                    vectSD[i] = mapTask.at(nCurTask).at(nStartTimeIdx) + mapTask.at(nCurTask).at(nServiceTimeIdx);
                }
                else
                {
                    vectSD[i] += mapTask.at(nCurTask).at(nServiceTimeIdx);
                }
            }
            if (j > 0)
            {
                TC += dist(mapTask.at(SolX.vectRoute[i][j - 1]), mapTask.at(nCurTask));
                vectSD[i] = vectSD[i] + dist(mapTask.at(SolX.vectRoute[i][j - 1]), mapTask.at(nCurTask));
                if (vectSD[i] < mapTask.at(nCurTask).at(nStartTimeIdx))
                {
                    WT += mapTask.at(nCurTask).at(nStartTimeIdx) - vectSD[i];
                    // if (WT == 0)
                    // {
                    //     std::cout << "WT is 0\n";
                    //     std::this_thread::sleep_for(50ms);
                    // }
                    vectSD[i] = mapTask.at(nCurTask).at(nStartTimeIdx) + mapTask.at(nCurTask).at(nServiceTimeIdx);
                }
                else
                {
                    vectSD[i] += mapTask.at(nCurTask).at(nServiceTimeIdx);
                }
            }
        }
        SD += vectSD[i];
    }
    SolX.TC = TC;
    SolX.SD = SD;
    SolX.NV = NV;
    SolX.WT = WT;
    return SolX;
}

float SACost(Sol SolX)
{
    int phi = 0.01 * SolX.TC;
    return SolX.TC + phi * SolX.WT;
}

double cost(Sol SolX, int alpha = 100000, int beta = 10000, int gamma = 1000, int delta = 100)
{
    return SolX.NV * alpha + SolX.TC * beta + SolX.SD * gamma + SolX.WT * delta;
}

float distCost(unordered_map<int, vector<int>> mapTask, vector<int> vctRoute)
{
    float TC = 0;
    for (int i = 0; i < vctRoute.size(); i++)
    {
        if (i == 0 || i == vctRoute.size() - 1)
        {
            TC += dist(mapTask.at(vctRoute.at(i)), mapTask.at(0));
        }
        else
        {
            TC += dist(mapTask.at(vctRoute.at(i - 1)), mapTask.at(vctRoute.at(i)));
        }
    }
    return TC;
}

unordered_map<int, float> saveDepotToX(unordered_map<int, vector<int>> mapTask)
{
    unordered_map<int, float> mapDistDepToX;
    for (auto itr = mapTask.begin(); itr != mapTask.end(); itr++)
    {
        mapDistDepToX[itr->first] = dist(mapTask.at(0), itr->second);
    }
    return mapDistDepToX;
}

bool checkRouteFeasibility(vector<int> vRoute, unordered_map<int, vector<int>> mapTask, int nMaxLoad)
{
    int nLoad = 0;
    float SD = 0;
    int nStartTimeIdx = 3;
    int nEndTimeIdx = 4;
    int nServiceTimeIdx = 5;
    int nDemand = 2;
    for (int i = 0; i < vRoute.size(); i++)
    {
        nLoad += mapTask.at(vRoute[i]).at(nDemand);
        if (nLoad > nMaxLoad)
        {
            return false;
        }
    }
    for (int i = 0; i < vRoute.size(); i++)
    {
        float nStartTime = mapTask.at(vRoute[i]).at(nStartTimeIdx);
        if (i == 0)
        {
            SD = max(dist(mapTask.at(0), mapTask.at(vRoute[i])), nStartTime);
        }
        else
        {
            float dis_i_j = dist(mapTask.at(vRoute[i - 1]), mapTask.at(vRoute[i]));
            SD = max(SD + mapTask.at(vRoute[i - 1]).at(nServiceTimeIdx) + dis_i_j, nStartTime);
        }
        if (SD > mapTask.at(vRoute[i]).at(nEndTimeIdx))
        {
            return false;
        }
    }
    SD += mapTask.at(vRoute.back()).at(nServiceTimeIdx) + dist(mapTask.at(vRoute.back()), mapTask.at(0));
    if (SD > mapTask.at(0).at(nEndTimeIdx))
    {
        return false;
    }
    return true;
}

Sol insertPDPair(Sol SolX, int pTask, unordered_map<int, vector<int>> mapTask, int nMaxLoad)
{
    double minCost = DBL_MAX;
    int nRouteIdx;
    vector<int> vRoute;
    int nDemanndIdx = 2;
    int nDeliveryIdx = 7;
    for (int i = 0; i < SolX.vectRoute.size(); i++)
    {
        for (int j = 0; j < SolX.vectRoute.at(i).size() + 1; j++)
        {
            vector<int> tempRoute = SolX.vectRoute[i];
            tempRoute.insert(tempRoute.begin() + j, pTask);
            for (int k = j + 1; k < tempRoute.size() + 1; k++)
            {
                tempRoute.insert(tempRoute.begin() + k, mapTask.at(pTask).at(nDeliveryIdx));
                if (checkRouteFeasibility(tempRoute, mapTask, nMaxLoad))
                {
                    float thisCost = distCost(mapTask, tempRoute);
                    if (minCost > thisCost)
                    {
                        minCost = thisCost;
                        vRoute = tempRoute;
                        nRouteIdx = i;
                    }
                }
                else
                {
                }
                tempRoute.erase(tempRoute.begin() + k);
            }
            tempRoute.erase(tempRoute.begin() + j);
        }
    }
    if (vRoute.empty())
    {
        vector<int> vNewRoute;
        vNewRoute.push_back(pTask);
        vNewRoute.push_back(mapTask.at(pTask).at(nDeliveryIdx));
        SolX.vectRoute.push_back(vNewRoute);
    }
    else
    {
        SolX.vectRoute.at(nRouteIdx) = vRoute;
    }
    return SolX;
}

vector<vector<int>> insertPDtoRoute(int pTask, vector<int> vRoute, unordered_map<int, vector<int>> mapTask, int nMaxLoad)
{
    vector<vector<int>> feasibleRoutes; // check why sets can't be used
    int nDeliveryIdx = 7;
    // std::cout << "check 10\n";
    for (int i = 0; i < vRoute.size() + 1; i++)
    {
        // std::cout << "check 11\n";
        vRoute.insert(vRoute.begin() + i, pTask);
        for (int j = i + 1; j < vRoute.size() + 1; j++)
        {
            // std::cout << "check 12\n";
            int dTask = mapTask.at(pTask).at(nDeliveryIdx);
            vRoute.insert(vRoute.begin() + j, dTask);
            // printVector(vRoute);
            if (checkRouteFeasibility(vRoute, mapTask, nMaxLoad))
            {
                feasibleRoutes.push_back(vRoute);
            }
            // std::cout << "check 12\n";
            vRoute.erase(vRoute.begin() + j);
        }
        vRoute.erase(vRoute.begin() + i);
    }
    return feasibleRoutes;
}

vector<vector<int>> generateRoutes(vector<int> route, unordered_map<int, vector<int>> mapTask, int nMaxLoad)
{
    vector<vector<int>> listRoutes;
    int nPickupIdx = 6;
    int nDeliveryIdx = 7;
    for (int i = 0; i < route.size(); i++)
    {
        // std::cout << route.at(i) << " is the task at route(" << i << ")\n";
        // std::cout << mapTask.at(route.at(i)).at(nPickupIdx) << " == 0?\n";

        if (mapTask.at(route.at(i)).at(nPickupIdx) == 0)
        {
            // std::cout << i << " i in if\n";
            int nSaveP = route.at(i);
            int nSavePIdx = i;
            int nSaveD;
            int nSaveDIdx;

            // std::cout << nSaveP << " nSaveP\n";
            // std::cout << nSavePIdx << " nSaveP\n";
            route.erase(route.begin() + nSavePIdx);
            for (int k = nSavePIdx; k < route.size(); k++)
            {
                if (mapTask.at(route.at(k)).at(nPickupIdx) == nSaveP)
                {
                    nSaveD = route.at(k);
                    nSaveDIdx = k;
                    break;
                }
            }
            // std::cout << nSaveD << " nSaveD\n";
            // std::cout << nSaveDIdx << " nSaveDIdx\n";
            route.erase(route.begin() + nSaveDIdx);

            for (int x = 0; x < route.size(); x++)
            {
                route.insert(route.begin() + x, nSaveP);
                for (int y = x + 1; y < route.size(); y++)
                {
                    route.insert(route.begin() + y, nSaveD);
                    // for (auto c : route)
                    //     std::cout << c << " ";
                    // std::cout << endl;
                    if (checkRouteFeasibility(route, mapTask, nMaxLoad))
                    {
                        listRoutes.push_back(route);
                    }
                    route.erase(route.begin() + y);
                }
                route.erase(route.begin() + x);
            }
            route.insert(route.begin() + nSaveDIdx, nSaveD);
            route.insert(route.begin() + nSavePIdx, nSaveP);
        }
    }

    return listRoutes;
}

void printSolution(Sol SolX, int maxVehicles)
{
    if (SolX.NV > maxVehicles)
        std::cout << "\nMax no of vechiles is set as " << maxVehicles << ". It is exceeded\n";
    std::cout << "\nObjective cost: " << cost(SolX) << endl;
    std::cout << "Number of vehicles: " << SolX.NV << endl;
    std::cout << "Total cost: " << SolX.TC << endl;
    std::cout << "Schedule duration: " << SolX.SD << endl;
    std::cout << "Waiting time: " << SolX.WT << endl;
    std::cout << "Routes" << endl;
    for (int i = 0; i < SolX.vectRoute.size(); i++)
    {
        std::cout << "  " << i + 1 << ": ";
        for (int j = 0; j < SolX.vectRoute.at(i).size(); j++)
        {
            std::cout << SolX.vectRoute.at(i).at(j) << " ";
        }
        std::cout << endl;
    }
    return;
}

set<Sol> PDShiftOperator(Sol SolX, unordered_map<int, vector<int>> mapTask, int nMaxLoad)
{
    int nPickupIdx = 6;
    int nDeliveryIdx = 7;
    set<Sol> nyborSols;
    vector<vector<int>> vectRoutes = SolX.vectRoute;
    // std::cout << "check 1" << endl;
    for (int i = 0; i < vectRoutes.size(); i++)
    { // visualise 2D matrix of routeNos -> to take pair of routes (see 2.)
        // std::cout << "check 2\n";
        for (int j = 0; j < vectRoutes.size(); j++)
        { // selects both (i,j) and (j,i)
            // std::cout << "check 3\n";
            if (i != j)
            { // taken all pairs of routes
                for (int k = 0; k < vectRoutes.at(i).size(); k++)
                { // NOTE: i/j
                    // needs a functions which takes one route and a PD pair and checks all possible feasible positions and returns all routes
                    // std::cout << "check 4\n";
                    if (mapTask.at(vectRoutes[i][k]).at(nPickupIdx) == 0)
                    { // choose P from route i and fix in route j
                        // std::cout << "check 5\n";
                        vector<vector<int>> feasibleRoutes = insertPDtoRoute(vectRoutes.at(i).at(k), vectRoutes.at(j), mapTask, nMaxLoad);
                        // std::cout << "check 6\n";
                        if (!feasibleRoutes.empty())
                        { //          if  there's feasible routes
                            // std::cout << "check 7\n";
                            int nSaveP = vectRoutes[i][k];
                            vectRoutes[i].erase(vectRoutes[i].begin() + k);
                            int nSaveD;
                            int nDIdxOfK;

                            for (int l = k; l < vectRoutes.at(i).size(); l++)
                            {
                                if (mapTask.at(vectRoutes[i][l]).at(nPickupIdx) == nSaveP)
                                {
                                    nDIdxOfK = l;
                                    nSaveD = vectRoutes[i][l];
                                    break;
                                }
                            }

                            vectRoutes[i].erase(vectRoutes[i].begin() + nDIdxOfK);

                            for (auto it = feasibleRoutes.begin(); it != feasibleRoutes.end(); it++)
                            {
                                // std::cout << "check 8\n";
                                Sol SolY = SolX;
                                SolY.vectRoute.at(i) = vectRoutes.at(i);
                                SolY.vectRoute.at(j) = *it;
                                SolY = populateSolution(SolY, mapTask);
                                // printSolution(SolY, 20);
                                nyborSols.insert(SolY);
                                // if (nyborSols.size() == nOfSols)
                                //     return nyborSols;
                            }

                            vectRoutes[i].insert(vectRoutes[i].begin() + nDIdxOfK, nSaveD);
                            vectRoutes[i].insert(vectRoutes[i].begin() + k, nSaveP);
                        }
                    }
                }
            }
        }
    }
    return nyborSols;
}

set<Sol> PDExchangeOperator(Sol SolX, /*int nOfSols,*/ unordered_map<int, vector<int>> mapTask, int nMaxLoad)
{

    int nPickupIdx = 6;
    int nDeliveryIdx = 7;
    set<Sol> nyborSols;
    vector<vector<int>> vectRoutes = SolX.vectRoute;
    // std::cout << "check 1" << endl;
    for (int i = 0; i < vectRoutes.size(); i++)
    { // visualise 2D matrix of routeNos -> to take pair of routes (see 2.)
        // std::cout << "check 2\n";
        for (int j = 0; j < vectRoutes.size(); j++)
        { // selects both (i,j) and (j,i)
            // std::cout << "check 3\n";
            if (i != j)
            { // taken all pairs of routes
                for (int k = 0; k < vectRoutes.at(i).size(); k++)
                {
                    int nCurElemntInI = vectRoutes.at(i).at(k);
                    if (mapTask.at(nCurElemntInI).at(nPickupIdx) == 0)
                    { // if pickup: we need the P to take the D
                        int nSavePi = nCurElemntInI;
                        int nSavePiIdx = k;
                        // remove one pair
                        int nSaveDi;
                        int nSaveDiIdx;

                        vectRoutes.at(i).erase(vectRoutes.at(i).begin() + nSavePiIdx);
                        for (int x = nSavePiIdx; x < vectRoutes.at(i).size(); x++)
                        {
                            if (mapTask.at(vectRoutes.at(i).at(x)).at(nPickupIdx) == nSavePi)
                            {
                                nSaveDi = vectRoutes.at(i).at(x);
                                nSaveDiIdx = x;
                                break;
                            }
                        }
                        vectRoutes.at(i).erase(vectRoutes.at(i).begin() + nSaveDiIdx);

                        for (int l = 0; l < vectRoutes.at(j).size(); l++)
                        { // equivalent ot for k loop
                            int nCurElemntInJ = vectRoutes.at(j).at(l);
                            if (mapTask.at(nCurElemntInJ).at(nPickupIdx) == 0)
                            {
                                int nSavePj = nCurElemntInJ;
                                int nSavePjIdx = l;
                                int nSaveDj;
                                int nSaveDjIdx;

                                vectRoutes.at(j).erase(vectRoutes.at(j).begin() + nSavePjIdx);
                                for (int x = nSavePjIdx; x < vectRoutes.at(j).size(); x++)
                                {
                                    if (mapTask.at(vectRoutes.at(j).at(x)).at(nPickupIdx) == nSavePj)
                                    {
                                        nSaveDj = vectRoutes.at(j).at(x);
                                        nSaveDjIdx = x;
                                        break;
                                    }
                                }
                                vectRoutes.at(j).erase(vectRoutes.at(j).begin() + nSaveDjIdx);
                                // do task , call func

                                vector<int> routeI = vectRoutes.at(i);
                                vector<int> routeJ = vectRoutes.at(j);

                                vector<vector<int>> feasibleRouteIs = insertPDtoRoute(nSavePj, routeI, mapTask, nMaxLoad);
                                vector<vector<int>> feasibleRouteJs = insertPDtoRoute(nSavePi, routeJ, mapTask, nMaxLoad);

                                if (!feasibleRouteIs.empty() && !feasibleRouteJs.empty())
                                {
                                    for (int a = 0; a < feasibleRouteIs.size(); a++)
                                    {
                                        for (int b = 0; b < feasibleRouteJs.size(); b++)
                                        {
                                            Sol SolY = SolX;
                                            SolY.vectRoute.at(i) = feasibleRouteIs.at(a);
                                            SolY.vectRoute.at(j) = feasibleRouteJs.at(b);
                                            SolY = populateSolution(SolY, mapTask);
                                            // printSolution(SolY, 20);
                                            nyborSols.insert(SolY);
                                            // if (nyborSols.size() == nOfSols)
                                            //     return nyborSols;
                                        }
                                    }
                                }

                                vectRoutes.at(j).insert(vectRoutes.at(j).begin() + nSaveDjIdx, nSaveDj);
                                vectRoutes.at(j).insert(vectRoutes.at(j).begin() + nSavePjIdx, nSavePj);
                            }
                        }

                        vectRoutes.at(i).insert(vectRoutes.at(i).begin() + nSaveDiIdx, nSaveDi);
                        vectRoutes.at(i).insert(vectRoutes.at(i).begin() + nSavePiIdx, nSavePi);
                        // insert that pair
                    }
                }
            }
        }
    }
    return nyborSols;
}

set<Sol> PDRearrangeOperator(Sol SolX, /*int nOfSols,*/ unordered_map<int, vector<int>> mapTask, int nMaxLoad)
{
    set<Sol> nyborSols;
    // set<float> setCosts;
    vector<vector<int>> feasibleRoutes;
    for (int i = 0; i < SolX.vectRoute.size(); i++)
    {
        float minCost = DBL_MAX;
        Sol bestSol;
        feasibleRoutes = generateRoutes(SolX.vectRoute.at(i), mapTask, nMaxLoad);
        if (!feasibleRoutes.empty())
        {
            for (int j = 0; j < feasibleRoutes.size(); j++)
            {
                Sol SolY = SolX;
                SolY.vectRoute.at(i) = feasibleRoutes.at(j);
                SolY = populateSolution(SolY, mapTask);
                double thisCost = cost(SolY);
                // std::cout << "mincost " << minCost << endl;
                // std::cout << "thiscost " << thisCost << endl;
                if (thisCost < minCost)
                {
                    bestSol = SolY;
                    minCost = thisCost;
                }
            }
            nyborSols.insert(bestSol);
            // if (nyborSols.size() == nOfSols)
            //     return nyborSols;
        }
        else
        {
            nyborSols.insert(SolX);
            // if (nyborSols.size() == nOfSols)
            //     return nyborSols;
        }
    }
    return nyborSols;
}

Sol DSL(Sol SolX, int /* 0:SE, !0: R*/ method, int nOfTimes, unordered_map<int, vector<int>> mapTask, int nMaxLoad)
{
    Sol Sb = SolX, Sb1;
    float costSb = cost(Sb);
    float costSb1 = FLT_MAX;
    int i = 0;
    set<Sol> NeighborhoodSols;

    do
    {
        if (method == 0)
        {
            set<Sol> PDShiftNeighbourSols = PDShiftOperator(Sb, mapTask, nMaxLoad);
            set<Sol> PDExchangeNeighbourSols = PDExchangeOperator(Sb, mapTask, nMaxLoad);

            // cout << "PDShiftNeighbourSols " << PDShiftNeighbourSols.size()<< endl;
            // cout << "PDExchangeNeighbourSols " << PDExchangeNeighbourSols.size() << endl;

            NeighborhoodSols.insert(PDShiftNeighbourSols.begin(), PDShiftNeighbourSols.end());
            NeighborhoodSols.insert(PDExchangeNeighbourSols.begin(), PDExchangeNeighbourSols.end());
        }
        else
        {
            NeighborhoodSols = PDRearrangeOperator(Sb, mapTask, nMaxLoad);
        }

        // cout << "NeighborhoodSols " << NeighborhoodSols.size() << endl;

        for (auto it = NeighborhoodSols.begin(); it != NeighborhoodSols.end(); it++)
        {
            if (cost(*it) < costSb)
            {
                costSb1 = cost(*it);
                Sb1 = *it;
            }
        }

        if (costSb1 > costSb)
        {
            return Sb;
        }

        Sb = Sb1;
        costSb = costSb1;

        i++;
    } while (i < nOfTimes);

    return Sb;
}

Sol metropolis(Sol SolX, int T, float T0, float delta, set<Sol> &tabuSet, unordered_map<int, vector<int>> mapTask, int nMaxLoad)
{
    // Sol S1;
    set<Sol> PDShiftNeighbourSols = PDShiftOperator(SolX, mapTask, nMaxLoad);
    set<Sol> PDExchangeNeighbourSols = PDExchangeOperator(SolX,mapTask, nMaxLoad);
    set<Sol> UnionOfShitExchangeSets;

    // std::cout << "Size of PDShiftOperator set is " << PDShiftNeighbourSols.size() << endl;
    // std::cout << "Size of PDExchangeOperator set is " << PDExchangeNeighbourSols.size() << endl;

    UnionOfShitExchangeSets.insert(PDShiftNeighbourSols.begin(), PDShiftNeighbourSols.end());
    UnionOfShitExchangeSets.insert(PDExchangeNeighbourSols.begin(), PDExchangeNeighbourSols.end());

    const double e = 2.718281828;
    float prob;

    // std::cout << "Size of UnionSE set is " << UnionOfShitExchangeSets.size() << endl;

    while (T > T0)
    {
        int k = 0;
        Sol randSol;
        int i = 0;
        for (auto it = UnionOfShitExchangeSets.begin(); it != UnionOfShitExchangeSets.end(); it++)
        {
            if (tabuSet.find(*it) == tabuSet.end())
            { // not in tabuset
                randSol = *it;
                break;
            }
            ++i;
        }

        if (i == UnionOfShitExchangeSets.size())
        {
            std::cout << "Check metropolis: random solution generated is present in tabuset\ntry increasing the no of solns\n";
        }

        // std::cout << "Val i " << i << endl;

        float DETLTA = SACost(randSol) - SACost(SolX);
        if (DETLTA <= 0)
        {
            prob = 1;
        }
        else
        {
            prob = pow(e, -(DETLTA / T));
        }
        if ((float)rand() / (float)RAND_MAX <= prob)
        {
            tabuSet.insert(randSol);
            return randSol;
        }
        T = delta * T;
        // std::cout << k++ << endl;
    }

    // std::cout << "Metropolis not working\n";
    return SolX;
}

// Sol tabuEmbeddedSA(Sol SolX, int MSNI, int nOfTimes, int nOfSols = 10, int T, int T0, float delta, set<Sol> &tabuSet, unordered_map<int, vector<int>> mapTask, int nMaxLoad)
// {
//     Sol Sb = DSL(SolX, 0, nOfTimes, nOfSols, mapTask, nMaxLoad);
//     Sb = DSL(SolX, 1, nOfTimes, nOfSols, mapTask, nMaxLoad);
//     int NoImpr = 0;
//     Sol S = Sb;
//     Sol S1;
//     while (NoImpr < MSNI)
//     {
//         // Step 5.1
//         S1 = metropolis(S, nOfSols, T, T0, delta, tabuSet, mapTask, nMaxLoad);
//         vector<vector<int>> vS = S.vectRoute;
//         vector<vector<int>> vS1 = S1.vectRoute;
//         int n = min(vS.size(), vS1.size());
//         vector<int> recordRoute;

//         // Step 5.1 recording routes (why just 2 routes? what if there are 0 or >2?)
//         for (int i = 0; i < n; i++) {
//             if (!equal(vS.at(i).begin(), vS.at(i).begin() + n, vS1.begin())) {
//                 recordRoute.push_back(i);
//             }
//             if (recordRoute.size() == 2) {
//                 break;
//             }
//         }

//         // Step 5.2 reduce no of vehicles, where to save the solution after reducing the no of vehicles?
//         set<Sol> PDShift = PDShiftOperator(S1, nOfSols, mapTask, nMaxLoad);
//         Sol bestObjCostSol; // saved in here, but this solution in not beign used.
//         float bestCost = FLT_MAX;

//         for (auto it = PDShift.begin(); it != PDShift.end(); it++) {
//             if (cost(*it) < bestCost) {
//                 bestCost = cost(*it);
//                 bestObjCostSol = *it;
//             }
//         }

//         if (bestCost == FLT_MAX) {
//             std::cout << " Step 5.2: No mincost obtained\n";
//         }
//         else {
//             // S1 = bestObjCostSol;
//             /* result of 5.2 is written back on S1
//             * but this will collide with the write of 5.3 on 5.1
//             */
//            std::cout << " Step 5.2: Nothing done with the result of this code block\n";
//         }

//         // Step 5.3 is done on 5.1, what to do with the result of 5.2?
//         if (recordRoute.size() >= 2) {
//             // ______________________RESUME______________________
//             //Take the two routes; rearrange by best tc; insert into S1
//             // S1.vectRoute.at(0) =
//         }
//         else {
//             std::cout << " Step 5.3: No two modified routes after PDS/PDE is found\n";
//         }
//         Sol S1b = DSL(S1, 0, nOfTimes, nOfSols, mapTask, nMaxLoad);
//         S1b = DSL(S1b, 1, nOfTimes, nOfSols, mapTask, nMaxLoad);

//         if (cost(S1b) < cost(Sb)) {
//             Sb = S1b;
//             NoImpr = 0;
//         }
//         else {
//             NoImpr += 1;
//         }
//         S = S1b;
//     }
//     return Sb;
// }

vector<Sol> bestOfX(set<Sol> setSol, int count, unordered_map<int, vector<int>> mapTask, int nMaxLoad) {
    vector<pair<float, Sol>> vectPairTCSol;
    for (auto it = setSol.begin(); it != setSol.end(); it++) {
        vectPairTCSol.push_back(make_pair(it->TC, *it));
    }

    sort(vectPairTCSol.begin(), vectPairTCSol.end());

    vector<Sol> result;
    int s = vectPairTCSol.size();
    // std::cout << count;
    int n = min(count, s);
    for (auto it = vectPairTCSol.begin(); it < vectPairTCSol.begin() + n; it++) {
        result.push_back(it->second);
    }

    return result;
}

Sol createInitialSolution(unordered_map<int, vector<int>> mapTask, int method, unordered_set<int> setTask, int nMaxLoad)
{
    int nDepot = 0;
    int nInitPickup;
    int nInitDelivery;
    int nEndTimeIdx = 4;
    int nPickupIdx = 6;
    int nDeliveryIdx = 7;
    int nEarliestTaskTime = INT_MAX;
    int nEarliestTaskNo = INT_MAX;
    Sol Solution;
    Sol SolX;

    // farthest combined round trip distance to the depot (Depo -> P1 -> D1 -> Depo)
    if (method == 0)
    {
        float fMaxRoundTrip = 0; // saves max round trip distance
        for (auto itr = mapTask.begin(); itr != mapTask.end(); itr++)
        {
            if (itr->first != 0 && itr->second.at(nPickupIdx) == 0)
            {                                                                                         // select only non depo and pickup task
                float fDepotToP = dist(mapTask.at(nDepot), itr->second);                              // distance: depot to Pickup
                float fPToD = dist(itr->second, mapTask.at(itr->second.at(nDeliveryIdx)));            // distance: pickup to delivery
                float fDToDepot = dist(mapTask.at(itr->second.at(nDeliveryIdx)), mapTask.at(nDepot)); // distance: delivery to depot
                float fCurRoundTrip = fDepotToP + fPToD + fDToDepot;                                  // round trip distance

                if (fCurRoundTrip > fMaxRoundTrip)
                {
                    fMaxRoundTrip = fCurRoundTrip;
                    nInitPickup = itr->first;
                    nInitDelivery = itr->second.at(nDeliveryIdx);
                }
            }
        }

        vector<int> vInitRoute;
        vInitRoute.push_back(nInitPickup);
        vInitRoute.push_back(nInitDelivery);
        SolX.vectRoute.push_back(vInitRoute);
        setTask.erase(nInitPickup);
        setTask.erase(nInitDelivery);

        vector<pair<float, int>> rtDistanceToTask; // saves the <end time,delivery locations>
        for (auto it = mapTask.begin(); it != mapTask.end(); it++)
        {
            if (it->first != 0 && it->second.at(nPickupIdx) == 0)
            {
                float fDepotToP = dist(mapTask.at(nDepot), it->second);                              // distance: depot to Pickup
                float fPToD = dist(it->second, mapTask.at(it->second.at(nDeliveryIdx)));             // distance: pickup to delivery
                float fDToDepot = dist(mapTask.at(it->second.at(nDeliveryIdx)), mapTask.at(nDepot)); // distance: delivery to depot
                float fCurRoundTrip = fDepotToP + fPToD + fDToDepot;
                rtDistanceToTask.push_back(make_pair(fCurRoundTrip, it->first));
            }
        }

        sort(rtDistanceToTask.begin(), rtDistanceToTask.end());
        reverse(rtDistanceToTask.begin(), rtDistanceToTask.end());

        float thisCost;
        while (!setTask.empty())
        {
            int nSolTaskNo;
            int nIdx;
            float minCost = FLT_MAX;
            // after exisiting the the size of rtDistanceToTask reduces by one
            for (int i = 0; i < rtDistanceToTask.size(); i++)
            {
                if (setTask.find(rtDistanceToTask[i].second) != setTask.end())
                {
                    int nPickupTask = rtDistanceToTask[i].second;
                    Sol SolY = insertPDPair(SolX, nPickupTask, mapTask, nMaxLoad);
                    SolY = populateSolution(SolY, mapTask);
                    thisCost = SolY.TC;
                    if (thisCost < minCost)
                    {
                        minCost = thisCost;
                        nSolTaskNo = nPickupTask;
                        nIdx = i;
                        Solution = SolY;
                    }
                }
            }
            SolX = Solution;
            setTask.erase(nSolTaskNo);
            setTask.erase(mapTask.at(nSolTaskNo).at(nDeliveryIdx));
            rtDistanceToTask.erase(rtDistanceToTask.begin() + nIdx);
        }
    }
    else
    {
        /* Earliest entime first*/
        for (auto itr = mapTask.begin(); itr != mapTask.end(); itr++)
        {
            if (itr->first != 0 && itr->second.at(nDeliveryIdx) == 0)
            {
                if (itr->second.at(nEndTimeIdx) < nEarliestTaskTime)
                {
                    nEarliestTaskTime = itr->second.at(nEndTimeIdx);
                    nInitPickup = itr->second.at(nPickupIdx);
                    nInitDelivery = itr->first;
                }
            }
        }

        vector<int> vInitRoute;
        vInitRoute.push_back(nInitPickup);
        vInitRoute.push_back(nInitDelivery);
        SolX.vectRoute.push_back(vInitRoute);
        setTask.erase(nInitPickup);
        setTask.erase(nInitDelivery);

        vector<pair<int, int>> endTimesToTask; // saves the <end time,delivery locations>
        for (auto it = mapTask.begin(); it != mapTask.end(); it++)
        {
            if (it->first != 0 && it->second.at(nDeliveryIdx) == 0)
            {
                endTimesToTask.push_back(make_pair(it->second.at(nEndTimeIdx), it->first));
            }
        }

        sort(endTimesToTask.begin(), endTimesToTask.end());

        float thisCost;
        while (!setTask.empty())
        {
            int nSolTaskNo;
            int nIdx;
            float minCost = FLT_MAX;
            // after exisiting the the size of endTimesToTask reduces by one
            for (int i = 0; i < endTimesToTask.size(); i++)
            {
                if (setTask.find(endTimesToTask[i].second) != setTask.end())
                {
                    int nPickupTask = mapTask.at(endTimesToTask[i].second).at(nPickupIdx);
                    Sol SolY = insertPDPair(SolX, nPickupTask, mapTask, nMaxLoad);
                    SolY = populateSolution(SolY, mapTask);
                    thisCost = SolY.TC;
                    if (thisCost < minCost)
                    {
                        minCost = thisCost;
                        nSolTaskNo = nPickupTask;
                        nIdx = i;
                        Solution = SolY;
                    }
                }
            }
            SolX = Solution;
            setTask.erase(nSolTaskNo);
            setTask.erase(mapTask.at(nSolTaskNo).at(nDeliveryIdx));
            endTimesToTask.erase(endTimesToTask.begin() + nIdx);
        }
    }

    return SolX;
}

void printData(unordered_map<int, vector<int>> mapTask, int maxVehicles, int nMaxLoad)
{
    std::cout << "\nVehicle limit: " << maxVehicles << endl;
    std::cout << "Capacity limit: " << nMaxLoad << endl;

    for (auto it = mapTask.begin(); it != mapTask.end(); it++)
    {
        std::cout << it->first << ": ";
        for (auto e : it->second)
            std::cout << e << " ";
        std::cout << endl;
    }

    return;
}

void readData(string strFileName, unordered_map<int, vector<int>> &mapTask, unordered_set<int> &setTask, int &nMaxVehicles, int &nMaxLoad)
{

    ifstream MyFile;
    MyFile.open(strFileName);
    string line;
    int i = 0;
    while (MyFile.good())
    {

        vector<int> value;
        getline(MyFile, line);
        istringstream iss(line);
        int j = 0;
        int key;

        do
        {
            int subs;
            iss >> subs;

            if (i == 0)
            {
                if (j == 0)
                {
                    nMaxVehicles = subs;
                }
                if (j == 1)
                {
                    nMaxLoad = subs;
                }
            }
            else
            {
                if (j == 0)
                {
                    key = subs;
                }
                else
                {
                    value.push_back(subs);
                }
            }
            ++j;
        } while (iss);

        if (i > 0)
        {
            mapTask[key] = value;
        }
        ++i;
    }

    MyFile.close();

    for (auto itr = mapTask.begin(); itr != mapTask.end(); itr++)
    {
        if (itr->first != 0)
        {
            setTask.insert(itr->first);
        }
    }
    if (setTask.size() % 2 == 1)
    {
        std::cout << "Solution requires even number of PD locations\n";
        // std::this_thread::sleep_for(1s);
    }

    return;
}

int main()
{
    unordered_map<int, vector<int>> mapTask;
    unordered_set<int> setTask;
    set<Sol> tabuSet;
    int nMaxVehicles, nMaxLoad;
    float delta = 0.9;

    string path = "C:/Abhishek_work/vysakh_data_check/pdp_100/lrc202.txt";
    readData(path, mapTask, setTask, nMaxVehicles, nMaxLoad);
    // printData(mapTask, nMaxVehicles, nMaxLoad);
    // 0: max round trip, non 0: earliest end time

    cout << "Running for file path: " ;
    cout << path << endl;

    Sol initSol = createInitialSolution(mapTask, 0, setTask, nMaxLoad);
    cout << "Initial Solution";
    printSolution(initSol, nMaxVehicles);
    cout << "---------------------------" << endl;


    set<Sol> afterMetro;
    Sol pass = initSol;
    for (int k = 0; k < 10; k++) {
        Sol sol = metropolis(pass, 80, 0.008, delta, tabuSet, mapTask, nMaxLoad);
        afterMetro.insert(sol);
        // printSolution(sol, nMaxVehicles);
        pass = DSL(sol, 1, 15, mapTask, nMaxLoad);
        // pass = sol;
    }

    cout << "---------------------------" << endl;
    vector<Sol> aftermetrosort = bestOfX( afterMetro, 5, mapTask, nMaxLoad);
    cout << "\nBest soln after metropolis without DSL using R together\n";
    printSolution(aftermetrosort.at(0), nMaxVehicles);


    Sol afterDSL = DSL(aftermetrosort.at(0), 0, 15, mapTask, nMaxLoad);
    cout << "\nAfter DSL using Initial_solution\n";
    printSolution(afterDSL, nMaxVehicles);
    cout << "---------------------------" << endl;

    afterDSL = DSL(afterDSL, 1, 15, mapTask, nMaxLoad);
    cout << "\nAfter DSL using R\n";
    printSolution(afterDSL, nMaxVehicles);
    cout << "---------------------------" << endl;

    set<Sol> sS = PDShiftOperator(afterDSL, mapTask, nMaxLoad);
    // std::cout << "\nSize of set sS is " << sS.size() << endl;
    vector<Sol> vS = bestOfX(sS, 5, mapTask, nMaxLoad);

    // std::cout << "\nBest 5 solutions after Shift operaftor, if there are 5.\n";
    // for (auto it = vS.begin(); it != vS.end(); it++) {
    //     printSolution(*it, nMaxVehicles);
    // }

    cout << "\nBest solution after PDS\n";
    printSolution(vS.at(0), nMaxVehicles);
    cout << "---------------------------" << endl;

    set<Sol> sE = PDExchangeOperator(vS.at(0), mapTask, nMaxLoad);
    // std::cout << "\nSize of set is sE is " << sE.size() << endl;
    vector<Sol> vE = bestOfX(sE, 5, mapTask, nMaxLoad);

    // std::cout << "\nBest 5 solutions after Exchange operaftor, if there are 5.\n";
    // for (auto it = vE.begin(); it != vE.end(); it++) {
    //     printSolution(*it, nMaxVehicles);
    // }

    cout << "\nBest solution after PDE\n";
    printSolution(vE.at(0), nMaxVehicles);
    cout << "---------------------------" << endl;

    set<Sol> sR = PDRearrangeOperator(vE.at(0), mapTask, nMaxLoad);
    // cout << "\nSize of set is sR is " << sR.size() << endl;
    vector<Sol> vR = bestOfX(sR, 5, mapTask, nMaxLoad);

    // cout << "\nBest 5 solutions after Rearrange operaftor, if there are 5.\n";
    // for (auto it = vR.begin(); it != vR.end(); it++) {
    //     printSolution(*it, nMaxVehicles);
    // }

    cout << "\nBest solution after PDR\n";
    printSolution(vR.at(0), nMaxVehicles);
    cout << "---------------------------" << endl;



    set<Sol> afterMetro_re;
    Sol pass_re = vR.at(0);
    for (int k = 0; k < 10; k++) {
        Sol sol = metropolis(pass_re, 80, 0.008, delta, tabuSet, mapTask, nMaxLoad);
        afterMetro_re.insert(sol);
        // printSolution(sol, nMaxVehicles);
        pass_re = DSL(sol, 1, 15, mapTask, nMaxLoad);
        // pass = sol;
    }

    cout << "---------------------------" << endl;
    vector<Sol> aftermetrosort_re = bestOfX( afterMetro_re, 5, mapTask, nMaxLoad);
    cout << "\nBest soln after metropolis and DSL using R together\n";
    printSolution(aftermetrosort_re.at(0), nMaxVehicles);

    cout << "Results for file path: " ;
    cout << path << endl;

    return 0;
}


