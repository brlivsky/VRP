/*
 * Params set are different
 * Reads a single text file.
 * PD pairs are sorted based on largest round trip distance and earliest deadline and used only least cost
 * Shift operator
 * Exchange operator
 * Rearrange operator
 * DSL
 * Metropolis
 * K-restart
 * Timestamp
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
#include <string>
#include <cmath>
#include <time.h>

using namespace std;

int nDemandIdx = 2;
int nStartTimeIdx = 3;
int nEndTimeIdx = 4;
int nServiceTimeIdx = 5;
int nPickupIdx = 6;
int nDeliveryIdx = 7;

struct Sol
{
    int NV;
    float TC;
    float SD;
    float WT;
    vector<vector<int>> vectRoute; 
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

float dist(vector<int> vTask1, vector<int> vTask2)
{
    float fVal = sqrt(pow(vTask1.at(0) - vTask2.at(0), 2) + pow(vTask1.at(1) - vTask2.at(1), 2));
    return floor(fVal * 100.0) / 100.0;
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

Sol populateSolution(Sol SolX, unordered_map<int, vector<int>> mapTask)
{
    for (int i = 0; i < SolX.vectRoute.size();)
    {
        if (SolX.vectRoute[i].size() == 0)
            SolX.vectRoute.erase(SolX.vectRoute.begin() + i);
        else
            ++i;
    }

    int NV = SolX.vectRoute.size();
    float TC = 0, SD = 0, WT = 0;
    vector<float> vectSD(SolX.vectRoute.size(), 0);
    
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

bool checkRouteFeasibility(vector<int> vRoute, unordered_map<int, vector<int>> mapTask, int nMaxLoad)
{
    int nLoad = 0;
    float SD = 0;
    for (int i = 0; i < vRoute.size(); i++)
    {
        nLoad += mapTask.at(vRoute[i]).at(nDemandIdx);
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
    for (int i = 0; i < vRoute.size() + 1; i++)
    {
        vRoute.insert(vRoute.begin() + i, pTask);
        for (int j = i + 1; j < vRoute.size() + 1; j++)
        {
            int dTask = mapTask.at(pTask).at(nDeliveryIdx);
            vRoute.insert(vRoute.begin() + j, dTask);
            if (checkRouteFeasibility(vRoute, mapTask, nMaxLoad))
            {
                feasibleRoutes.push_back(vRoute);
            }
            vRoute.erase(vRoute.begin() + j);
        }
        vRoute.erase(vRoute.begin() + i);
    }
    return feasibleRoutes;
}

vector<vector<int>> generateRoutes(vector<int> route, unordered_map<int, vector<int>> mapTask, int nMaxLoad)
{
    vector<vector<int>> listRoutes;
    for (int i = 0; i < route.size(); i++)
    {
        if (mapTask.at(route.at(i)).at(nPickupIdx) == 0)
        {
            int nSaveP = route.at(i);
            int nSavePIdx = i;
            int nSaveD;
            int nSaveDIdx;

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
            route.erase(route.begin() + nSaveDIdx);

            for (int x = 0; x < route.size(); x++)
            {
                route.insert(route.begin() + x, nSaveP);
                for (int y = x + 1; y < route.size(); y++)
                {
                    route.insert(route.begin() + y, nSaveD);
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
        std::cout << "\nMax no of vechiles allocated has exceeded the limit\n";
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
    set<Sol> nyborSols;
    vector<vector<int>> vectRoutes = SolX.vectRoute;
    for (int i = 0; i < vectRoutes.size(); i++)
    { 
        for (int j = 0; j < vectRoutes.size(); j++)
        { 
            if (i != j)
            { 
                for (int k = 0; k < vectRoutes.at(i).size(); k++)
                { 
                    if (mapTask.at(vectRoutes[i][k]).at(nPickupIdx) == 0)
                    { 
                        vector<vector<int>> feasibleRoutes = insertPDtoRoute(vectRoutes.at(i).at(k), vectRoutes.at(j), mapTask, nMaxLoad);
                        if (!feasibleRoutes.empty())
                        { 
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
                                Sol SolY = SolX;
                                SolY.vectRoute.at(i) = vectRoutes.at(i);
                                SolY.vectRoute.at(j) = *it;
                                SolY = populateSolution(SolY, mapTask);
                                nyborSols.insert(SolY);
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
    set<Sol> nyborSols;
    vector<vector<int>> vectRoutes = SolX.vectRoute;
    for (int i = 0; i < vectRoutes.size(); i++)
    { 
        for (int j = 0; j < vectRoutes.size(); j++)
        { 
            if (i != j)
            { 
                for (int k = 0; k < vectRoutes.at(i).size(); k++)
                {
                    int nCurElemntInI = vectRoutes.at(i).at(k);
                    if (mapTask.at(nCurElemntInI).at(nPickupIdx) == 0)
                    { 
                        int nSavePi = nCurElemntInI;
                        int nSavePiIdx = k;
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
                        { 
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
                                            nyborSols.insert(SolY);
                                        }
                                    }
                                }
                                vectRoutes.at(j).insert(vectRoutes.at(j).begin() + nSaveDjIdx, nSaveDj);
                                vectRoutes.at(j).insert(vectRoutes.at(j).begin() + nSavePjIdx, nSavePj);
                            }
                        }
                        vectRoutes.at(i).insert(vectRoutes.at(i).begin() + nSaveDiIdx, nSaveDi);
                        vectRoutes.at(i).insert(vectRoutes.at(i).begin() + nSavePiIdx, nSavePi);
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
                if (thisCost < minCost)
                {
                    bestSol = SolY;
                    minCost = thisCost;
                }
            }
            nyborSols.insert(bestSol);
        }
        else
        {
            nyborSols.insert(SolX);
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
    const double e = 2.718281828;
    float prob;
    set<Sol> PDShiftNeighbourSols = PDShiftOperator(SolX, mapTask, nMaxLoad);
    set<Sol> PDExchangeNeighbourSols = PDExchangeOperator(SolX, mapTask, nMaxLoad);
    set<Sol> UnionOfShitExchangeSets;

    UnionOfShitExchangeSets.insert(PDShiftNeighbourSols.begin(), PDShiftNeighbourSols.end());
    UnionOfShitExchangeSets.insert(PDExchangeNeighbourSols.begin(), PDExchangeNeighbourSols.end());

    while (T > T0)
    {
        Sol randSol;
        int randomCheckLimit  = 20;
        int i;
        const int range_from  = 0;
        const int range_to    = UnionOfShitExchangeSets.size() - 1;

        for (i = 0; i < randomCheckLimit; i++) 
        {
            std:random_device                  rand_dev;
            std::mt19937                        generator(rand_dev());
            std::uniform_int_distribution<int>  distr(range_from, range_to);
            int r = distr(generator);
            auto it = UnionOfShitExchangeSets.begin();
            advance(it, r);
            if (tabuSet.find(*it) == tabuSet.end()) 
            { 
                randSol = *it;
                break;
            }          
        }

        if (i == randomCheckLimit)
        {
            std::cout << "Random solution generated is present in Tabuset\nIncrease randomCheckLimit > 20\n";
            randSol = SolX;
        }

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
    }

    return SolX;
}

vector<Sol> bestOfX(set<Sol> setSol, bool method /*0:TC | 1:cost*/, int count, unordered_map<int, vector<int>> mapTask, int nMaxLoad)
{
    vector<pair<float, Sol>> vectPairTCSol;

    for (auto it = setSol.begin(); it != setSol.end(); it++)
    {
        if (method == 0)
            vectPairTCSol.push_back(make_pair(it->TC, *it));
        else
            vectPairTCSol.push_back(make_pair(cost(*it), *it));
    }
    sort(vectPairTCSol.begin(), vectPairTCSol.end());

    vector<Sol> result;
    int s = vectPairTCSol.size();
    int n = min(count, s);

    for (auto it = vectPairTCSol.begin(); it < vectPairTCSol.begin() + n; it++)
    {
        result.push_back(it->second);
    }

    return result;
}

Sol createInitialSolution(unordered_map<int, vector<int>> mapTask, /*0:Max Comb RTT | 1:Min DL*/ int method, unordered_set<int> setTask, int nMaxLoad)
{
    Sol SolX;
    Sol Solution;    
    int nDepot = 0;
    int nInitPickup;
    int nInitDelivery;
    int nEarliestTaskTime = INT_MAX;
    int nEarliestTaskNo = INT_MAX;

    if (method == 0)
    {
        float fMaxRoundTrip = 0; 
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
        SolX = populateSolution(SolX, mapTask);
        setTask.erase(nInitPickup);
        setTask.erase(nInitDelivery);

        vector<pair<float, int>> rtDistanceToTask;
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
        SolX = populateSolution(SolX, mapTask);
        setTask.erase(nInitPickup);
        setTask.erase(nInitDelivery);

        vector<pair<int, int>> endTimesToTask; 
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
    }

    return;
}

int main()
{
    clock_t tStart = clock();
    unordered_map<int, vector<int>> mapTask;
    unordered_set<int> setTask;
    int nMaxVehicles, nMaxLoad;
    string path = "D:/Projects/VRPTW/lr101.txt";

    cout << path << endl;
    readData(path, mapTask, setTask, nMaxVehicles, nMaxLoad);
    cout << "\n---------------------------" << endl;
    cout << (int)(clock() - tStart) / CLOCKS_PER_SEC << ": Read the input file.\n";

    Sol S = createInitialSolution(mapTask, 0, setTask, nMaxLoad);
    cout << (int)(clock() - tStart) / CLOCKS_PER_SEC << ": Initial solution created.\n";

    printSolution(S, 25);
/*
    S = DSL(S, 0, 15, mapTask, nMaxLoad);
    cout << (int)(clock() - tStart) / CLOCKS_PER_SEC << ": DSL with NPD U NPE executed.\n";

    S = DSL(S, 1, 15, mapTask, nMaxLoad);
    cout << (int)(clock() - tStart) / CLOCKS_PER_SEC << ": DSL with NPR executed.\n";

    set<Sol> tabuSet;
    set<Sol> afterMetro;
    int noImpr = 0;
    int MSNI = 3;
    int K = 4;
    int T = 100;
    int T0 = 1;
    float delta = 0.9;

    cout << (int)(clock() - tStart) / CLOCKS_PER_SEC << ": Tabu Embedded SA executing...\n";

    for (int i = 0, k = 0; i < 10 && k < K; ++i)
    {
        Sol Sb = metropolis(S, T, T0, delta, tabuSet, mapTask, nMaxLoad);
        Sb = DSL(Sb, 0, 15, mapTask, nMaxLoad);
        Sb = DSL(Sb, 1, 15, mapTask, nMaxLoad);
        afterMetro.insert(Sb);

        if (cost(Sb) < cost(S))
        {
            noImpr = 0;
        }
        else
        {
            ++noImpr;
            if (noImpr >= MSNI)
            {
                ++k;
                i = -1;
                noImpr = 0;
                S = bestOfX(afterMetro, 1, MSNI, mapTask, nMaxLoad).at(0);
                if (k < K)
                    cout << (int)(clock() - tStart) / CLOCKS_PER_SEC << ": No improvement, restarting - " << k << endl;
                continue;
            }
        }
        S = Sb;
        cout << (int)(clock() - tStart) / CLOCKS_PER_SEC << ": Loop - " << i + 1 << endl;
    }

    cout << (int)(clock() - tStart) / CLOCKS_PER_SEC << ": Tabu Embedded SA completed.\n";
    cout << "---------------------------" << endl;

    vector<Sol> aftermetrosort = bestOfX(afterMetro, 1, 5, mapTask, nMaxLoad);
    cout << "\nBest solution:";
    printSolution(aftermetrosort.at(0), nMaxVehicles);
    */


    return 0;
}
