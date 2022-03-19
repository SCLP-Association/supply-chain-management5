package solver.lp;

import ilog.cplex.*;
import ilog.concert.*;
import java.util.ArrayList;

import java.io.File;
import java.io.FileNotFoundException;

import java.util.Scanner;

public class LPInstance
{
  // Supply Chain Management (SCM) Input Parameters
  int numCustomers;        		// the number of customers
  int numFacilities;           	// the number of facilities
  double[][] allocCostCF;   	// allocCostCF[c][f] is the service cost paid each time customer c is served by facility f
  double[] demandC;     		// demandC[c] is the demand of customer c
  double[] openingCostF;        // openingCostF[f] is the opening cost of facility f
  double[] capacityF;        	// capacityF[f] is the capacity of facility f
  int numMaxVehiclePerFacility; // maximum number of vehicles to use at an open facility
  double truckDistLimit;        // total driving distance limit for trucks
  double truckUsageCost;		// fixed usage cost paid if a truck is used
  double[][] distanceCF;        // distanceCF[c][f] is the roundtrip distance between customer c and facility f

  // IBM Ilog Cplex Solver
  IloCplex cplex;

  // Linear Programming (LP) Objective value
  double objectiveValue;

  public LPInstance(String fileName)
  {
    Scanner read = null;
    try
    {
      read = new Scanner(new File(fileName));
    } catch (FileNotFoundException e)
    {
      System.out.println("Error: in LPInstance() " + fileName + "\n" + e.getMessage());
      System.exit(-1);
    }

    numCustomers = read.nextInt();
    numFacilities = read.nextInt();
    numMaxVehiclePerFacility = numCustomers; // At worst case visit every customer with one vehicle

    System.out.println("numCustomers: " + numCustomers);
    System.out.println("numFacilities: " + numFacilities);
    System.out.println("numVehicle: " + numMaxVehiclePerFacility);

    allocCostCF = new double[numCustomers][];
    for (int i = 0; i < numCustomers; i++)
      allocCostCF[i] = new double[numFacilities];

    demandC = new double[numCustomers];
    openingCostF = new double[numFacilities];
    capacityF = new double[numFacilities];

    for (int i = 0; i < numCustomers; i++)
      for (int j = 0; j < numFacilities; j++)
        allocCostCF[i][j] = read.nextDouble();

    for (int i = 0; i < numCustomers; i++)
      demandC[i] = read.nextDouble();

    for (int i = 0; i < numFacilities; i++)
      openingCostF[i] = read.nextDouble();

    for (int i = 0; i < numFacilities; i++)
      capacityF[i] = read.nextDouble();

    truckDistLimit = read.nextDouble();
    truckUsageCost = read.nextDouble();

    distanceCF = new double[numCustomers][];
    for (int i = 0; i < numCustomers; i++)
      distanceCF[i] = new double[numFacilities];

    for (int i = 0; i < numCustomers; i++)
      for (int j = 0; j < numFacilities; j++)
        distanceCF[i][j] = read.nextDouble();
  }

  public void solve() throws IloException
  {
    try
    {
      cplex = new IloCplex();

      // decision variables
      IloNumVar[][][] fvc = new IloNumVar[this.numFacilities][this.numMaxVehiclePerFacility][this.numCustomers];
      IloNumVar[] factoryOpen = new IloNumVar[this.numFacilities];
      IloNumVar[][] factoryVehicles = new IloNumVar[this.numFacilities][this.numMaxVehiclePerFacility];

      for (int i = 0; i < this.numFacilities; i ++) {
        for (int j = 0; j < this.numMaxVehiclePerFacility; j ++) {
          for (int k = 0; k < this.numCustomers; k ++) {
            fvc[i][j][k] = cplex.numVar(0, 1, IloNumVarType.Float);
          }
          factoryVehicles[i][j] = cplex.numVar(0, 1, IloNumVarType.Float);
        }
        factoryOpen[i] = cplex.numVar(0, 1, IloNumVarType.Float);
      }

      //demand fulfilled by a factory should be less than its capacity
      for (int f = 0; f < this.numFacilities; f++) {
        IloNumExpr sumExpression1 = cplex.numExpr();
        for (int v = 0; v < this.numMaxVehiclePerFacility; v++){
          for (int c = 0; c < this.numCustomers; c++){
            sumExpression1 = cplex.sum(sumExpression1, cplex.prod(fvc[f][v][c], demandC[c]));
          }
        }
        cplex.addLe(sumExpression1, this.capacityF[f]);
      }


      //every customer served by at least one facility
      for (int c = 0; c < this.numCustomers; c++) {
        IloNumExpr sumExpression2 = cplex.numExpr();
        for (int f = 0; f < this.numFacilities; f++){
          for (int v = 0; v < this.numMaxVehiclePerFacility; v++){
            sumExpression2 = cplex.sum(sumExpression2, fvc[f][v][c]);
          }
        }
        cplex.addGe(sumExpression2, 1.0);
      }


      //vehicles don't drive too much
      for (int v = 0; v < this.numMaxVehiclePerFacility; v++) {
        IloNumExpr sumExpression3 = cplex.numExpr();
        for (int f = 0; f < this.numFacilities; f++) {
          for (int c = 0; c < this.numCustomers; c++) {
            sumExpression3 = cplex.sum(sumExpression3, cplex.prod(distanceCF[c][f], fvc[f][v][c]));
          }
        }
        cplex.addLe(sumExpression3, truckDistLimit);
      }

      //for consistency purposes
      for (int v = 0; v < this.numMaxVehiclePerFacility; v++) {
        for (int f = 0; f < this.numFacilities; f++) {
          for (int c = 0; c < this.numCustomers; c++) {
            cplex.addGe(factoryOpen[f], fvc[f][v][c]);
            cplex.addGe(factoryVehicles[f][v], fvc[f][v][c]);
          }
        }
      }

      IloNumExpr facilitiesCost = cplex.numExpr();
      IloNumExpr drivingCost = cplex.numExpr();
      IloNumExpr fixedVehicleCost = cplex.numExpr();

      //facilities cost
      for (int f = 0; f < this.numFacilities; f++) {
        facilitiesCost = cplex.sum(facilitiesCost, cplex.prod(factoryOpen[f], openingCostF[f]));
      }

      //driving cost
      for (int v = 0; v < this.numMaxVehiclePerFacility; v++) {
        for (int f = 0; f < this.numFacilities; f++) {
          for (int c = 0; c < this.numCustomers; c++) {
            drivingCost = cplex.sum(drivingCost, cplex.prod(fvc[f][v][c], allocCostCF[c][f]));
          }
        }
      }

      //fixed vehicle cost
      for (int v = 0; v < this.numMaxVehiclePerFacility; v++) {
        for (int f = 0; f < this.numFacilities; f++) {
          fixedVehicleCost = cplex.sum(fixedVehicleCost, factoryVehicles[f][v]);
        }
      }
      fixedVehicleCost = cplex.prod(fixedVehicleCost, truckUsageCost);

      //objective function
      cplex.addMinimize(cplex.sum(cplex.sum(facilitiesCost, drivingCost), fixedVehicleCost));

      if(cplex.solve())
      {
        this.objectiveValue = Math.ceil(cplex.getObjValue());

        // System.out.println("Meat:  " + cplex.getValue(vars[0]));
        // System.out.println("Bread:  " + cplex.getValue(vars[1]));
        // System.out.println("Objective value: " + cplex.getObjValue());
      }
      else
      {
        System.out.println("No Solution found!");
      }
    }
    catch(IloException e)
    {
      System.out.println("Error " + e);
    }
  }


}
