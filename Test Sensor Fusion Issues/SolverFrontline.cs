using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using SolverPlatform;
using ArtemisCore.Calculations;  // Reference to the Frontline Solver SDK

namespace SolverApp
{
  class cFLSsolver
  {

    List<cPose> boreas = new List<cPose>();
    List<cPose> measured = new List<cPose>();
    cTransform boreasBase = new cTransform(0, 0, 0, 180, 0, 152);
    cTransform boreasTool = new cTransform(0, 0, 0, 180, 0, 90);
    public cFLSsolver()
    {
      string filePath = @"C:\LocalDev\testSensorFusion\Book1.csv";
      string[] lines = System.IO.File.ReadAllLines(filePath);
      for (int i = 1; i < lines.Length; i++)
      {
        var line = lines[i];
        var parts = line.Split(',');
        boreas.Add(new cPose(0, 0, 0, double.Parse(parts[0]), double.Parse(parts[1]), double.Parse(parts[2])));
        measured.Add(new cPose(0, 0, 0, double.Parse(parts[3]), double.Parse(parts[4]), double.Parse(parts[5])));
      }
    }

    public void SolveMe()
    {
      double[] intitialValues = new double[] { boreasBase.rx.D2R(), boreasBase.ry.D2R(), boreasBase.rz.D2R(), boreasTool.rx.D2R(), boreasTool.ry.D2R(), boreasTool.rz.D2R() };
      solveIt(intitialValues, measured, boreas);
    }

    // Example objective: sum of squares of decision variables
    static (double Objective, double AverageError, double StdDev) ComputeMyObjective(double[] decisionValues, List<cPose> measured, List<cPose> boreas, bool baseOnly)
    {
      double sumsq = 0;
      List<double> individualErrors = new List<double>();

      // Initialize transforms using decision values.

      cTransform xbase = new cTransform(0, 0, 0, decisionValues[0].R2D(), decisionValues[1].R2D(), decisionValues[2].R2D());
      cTransform xtool = new cTransform(0, 0, 0, 180, 0, 90);
      if (!baseOnly)
      {
        xtool = new cTransform(0, 0, 0, decisionValues[3].R2D(), decisionValues[4].R2D(), decisionValues[5].R2D());
      }

      if (boreas.Count != measured.Count)
        throw new InvalidOperationException("Mismatch between boreas and measured data counts.");

      // Process each pair of poses.
      for (int i = 0; i < boreas.Count; i++)
      {
        // Transform boreasXYZ using xbase and xtool.
        cTransform boreasXYZ = ((xbase.getLHT() * boreas[i].getLHT()) * xtool.getLHT()).getTransformEulerXYZ();

        // Normalize angles to [-180, 180] before subtraction.
        double normRx = boreasXYZ.rx.m180p180();
        double normRy = boreasXYZ.ry.m180p180();
        double normRz = boreasXYZ.rz.m180p180();

        // Calculate the differences, normalizing each difference.
        double deltaRx = (normRx - measured[i].rx).m180p180();
        double deltaRy = (normRy - measured[i].ry).m180p180();
        double deltaRz = (normRz - measured[i].rz).m180p180();

        // Compute squared error for this measurement.
        double errorSquared = Math.Pow(deltaRx, 2) + Math.Pow(deltaRy, 2) + Math.Pow(deltaRz, 2);
        sumsq += errorSquared;

        // Save the individual error (root mean square for this point).
        double error = Math.Sqrt(errorSquared);
        individualErrors.Add(error);
      }

      // Overall objective value (e.g., the root of the total squared error).
      double objective = Math.Sqrt(sumsq);

      // Compute average error.
      double avgError = 0;
      foreach (var err in individualErrors)
        avgError += err;
      avgError /= individualErrors.Count;

      // Compute standard deviation of the errors.
      double variance = 0;
      foreach (var err in individualErrors)
        variance += Math.Pow(err - avgError, 2);
      variance /= individualErrors.Count;
      double stdDev = Math.Sqrt(variance);

      return (objective, avgError, stdDev);
    }


    static void solveIt(double[] initalValues, List<cPose> measured, List<cPose> boreas)
    {
      // Define number of decision variables and objectives
      int numVars = 6;
      int numObjectives = 1;
      bool baseOnly = true;

      // Create a Problem instance for minimization.
      using (Problem p = new Problem(Solver_Type.Minimize, numVars, numObjectives))
      {
        // Initialize decision variable values (e.g., all 1.0)
        for (int i = 0; i < numVars; i++)
        {
          p.VarDecision.InitialValue[i] = initalValues[i];
        }

        // Register the function evaluator to compute the objective value
        p.Evaluators[Eval_Type.Function].OnEvaluate += (Evaluator ev) =>
        {
          // Here you can read the current decision variable values from ev.Problem.VarDecision.Value
          double[] decisions = new double[numVars];

          for (int ii = 0; ii < numVars; ii++)
          {
            decisions[ii] = ev.Problem.VarDecision.Value[ii];
          }

          double currentObjective = ComputeMyObjective(decisions, measured, boreas, baseOnly).Objective;
          ev.Problem.FcnObjective.Value[0] = currentObjective;
          return Engine_Action.Continue;
        };

        // Optionally, set some engine parameters
        p.Engine.Params["MaxTime"].Value = 60.0;      // seconds
        p.Engine.Params["Iterations"].Value = 10000;      // maximum iterations
        p.Engine.Params["Precision"].Value = 1e-6;      // desired precision
        p.Engine = p.Engines[Engine.GRGName];
        p.ProblemType = Problem_Type.OptNLP;

        // Run the solver
        p.Solver.Optimize();
        baseOnly = false;
        for (int i = 0; i < numVars; i++)
        {
          p.VarDecision.InitialValue[i] = p.VarDecision.Value[i];
        }
        p.Solver.Optimize();
        for (int i = 0; i < numVars; i++)
        {
          p.VarDecision.InitialValue[i] = p.VarDecision.Value[i];
        }
        p.Solver.Optimize();
        for (int i = 0; i < numVars; i++)
        {
          p.VarDecision.InitialValue[i] = p.VarDecision.Value[i];
        }
        p.Solver.Optimize();
        for (int i = 0; i < numVars; i++)
        {
          p.VarDecision.InitialValue[i] = p.VarDecision.Value[i];
        }
        p.Solver.Optimize();
        for (int i = 0; i < numVars; i++)
        {
          p.VarDecision.InitialValue[i] = p.VarDecision.Value[i];
        }
        p.Solver.Optimize();


        // After optimization, get the optimized decision variable values.
        double[] optimizedValues = new double[numVars];
        for (int ii = 0; ii < numVars; ii++)
          optimizedValues[ii] = p.VarDecision.Value[ii];


        // Display the results
        Console.WriteLine("Optimization complete. Final objective: {0:F6}", p.FcnObjective.Value[0]);
        Console.WriteLine("Optimized decision variables:");
        for (int i = 0; i < optimizedValues.Length; i++)
        {
          Console.WriteLine("Variable {0}: {1:F6}", i, optimizedValues[i].R2D().m180p180());
        }

        var SolvedValues = ComputeMyObjective(optimizedValues, measured, boreas, baseOnly);
        Console.WriteLine($"SqrtSumSW {SolvedValues.Objective}");
        Console.WriteLine("Average Error: {0:F6}", SolvedValues.AverageError);
        Console.WriteLine("Standard Deviation: {0:F6}", SolvedValues.StdDev);
        Console.WriteLine("A3S: {0:F6}", SolvedValues.AverageError + 3 * SolvedValues.StdDev);

        cTransform boreasBase = new cTransform(0, 0, 0, optimizedValues[0].R2D().m180p180(), optimizedValues[1].R2D().m180p180(), optimizedValues[2].R2D().m180p180());
        cTransform boreasTool = new cTransform(0, 0, 0, optimizedValues[3].R2D().m180p180(), optimizedValues[4].R2D().m180p180(), optimizedValues[5].R2D().m180p180());

        // Process each pair of poses.
        for (int i = 0; i < boreas.Count; i++)
        {
          // Transform boreasXYZ using xbase and xtool.
          cTransform boreasXYZ = ((boreasBase.getLHT() * boreas[i].getLHT()) * boreasTool.getLHT()).getTransformEulerXYZ();

          // Normalize angles to [-180, 180] before subtraction.
          double normRx = boreasXYZ.rx.m180p180();
          double normRy = boreasXYZ.ry.m180p180();
          double normRz = boreasXYZ.rz.m180p180();

          // Calculate the differences, normalizing each difference.
          double deltaRx = (normRx - measured[i].rx).m180p180();
          double deltaRy = (normRy - measured[i].ry).m180p180();
          double deltaRz = (normRz - measured[i].rz).m180p180();

          // Compute squared error for this measurement.
          double errorSquared = Math.Pow(deltaRx, 2) + Math.Pow(deltaRy, 2) + Math.Pow(deltaRz, 2);

          Console.WriteLine("Pose {0}: Rx: {1:F6}, Ry: {2:F6}, Rz: {3:F6}, Error: {4:F6}", i, normRx, normRy, normRz, Math.Sqrt(errorSquared));

        }
      }

      // Pause before exiting (if running as a console app)
      Console.WriteLine("Press any key to exit.");
      Console.ReadKey();
    }
  }


}

