using Accord.Math;
using Accord.Math.Optimization;
using ArtemisCore.Calculations;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Optimization;
using MathNet.Numerics.Optimization.TrustRegion;
using Microsoft.VisualBasic;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Test_Sensor_Fusion_Issues
{
    internal class Solver
    {



        // Define the function to minimize: f(x) = (x - 3)^2
        public static void Solve(string dataFile)
        {
            List<cPose> boreas = new List<cPose>();
            List<cPose> measured = new List<cPose>();
            // Initialize transforms
            cTransform boreasBase = new cTransform(0, 0, 0, 0, 0, 0);
            cTransform boreasTool = new cTransform(0, 0, 0, 180, 0, 90);

            string[] data = System.IO.File.ReadAllLines(dataFile);
            
            for (int i = 1; i < data.Length; i++)
            {
                string s = data[i];
                string[] values = s.Split(',');
                boreas.Add(new cPose(0, 0, 0, double.Parse(values[0]), double.Parse(values[1]), double.Parse(values[2])));
                measured.Add(new cPose(0, 0, 0, double.Parse(values[3]), double.Parse(values[4]), double.Parse(values[5])));
            }

            // Objective function
            Func<Vector<double>, double> objectiveFunction = (Vector<double> x) =>
            {
                double sumsq = 0;

                // Initialize transforms
                cTransform xbase = new cTransform(0, 0, 0, x[0].R2D(), x[1].R2D(), x[2].R2D());
                cTransform xtool = new cTransform(0, 0, 0, x[3].R2D(), x[4].R2D(), x[5].R2D());

                // Ensure lists match in size
                if (boreas.Count != measured.Count)
                    throw new InvalidOperationException("Mismatch between boreas and measured data counts.");

                // Compute error sum
                for (int i = 0; i < boreas.Count; i++)
                {
                    // Transform boreasXYZ using xbase and xtool
                    cTransform boreasXYZ = (xbase.getLHT() * boreas[i].getLHT() * xtool.getLHT()).getTransformEulerXYZ();

                    // Normalize angles to [-180, 180] before subtraction
                    double normRx = boreasXYZ.rx.m180p180();
                    double normRy = boreasXYZ.ry.m180p180();
                    double normRz = boreasXYZ.rz.m180p180();

                    double deltaRx = (normRx - measured[i].rx).m180p180();
                    double deltaRy = (normRy - measured[i].ry).m180p180();
                    double deltaRz = (normRz - measured[i].rz).m180p180();

                    // Sum of squared errors
                    sumsq += Math.Pow(deltaRx, 2) + Math.Pow(deltaRy, 2) + Math.Pow(deltaRz, 2);
                }
                return Math.Sqrt(sumsq);
            };

            Func<Vector<double>, double> objectiveFunctionBaseOnly = (Vector<double> x) =>
            {
                double sumsq = 0;

                cTransform xbase = new cTransform(0, 0, 0, x[0].R2D(), x[1].R2D(), x[2].R2D());
                cTransform xtool = boreasTool;

                // Ensure lists match in size
                if (boreas.Count != measured.Count)
                    throw new InvalidOperationException("Mismatch between boreas and measured data counts.");

                // Compute error sum
                for (int i = 0; i < boreas.Count; i++)
                {
                    // Transform boreasXYZ using xbase and xtool
                    cTransform boreasXYZ = (xbase.getLHT() * boreas[i].getLHT() * xtool.getLHT()).getTransformEulerXYZ();

                    // Normalize angles to [-180, 180] before subtraction
                    double normRx = boreasXYZ.rx.m180p180();
                    double normRy = boreasXYZ.ry.m180p180();
                    double normRz = boreasXYZ.rz.m180p180();

                    double deltaRx = (normRx - measured[i].rx).m180p180();
                    double deltaRy = (normRy - measured[i].ry).m180p180();
                    double deltaRz = (normRz - measured[i].rz).m180p180();

                    // Sum of squared errors
                    sumsq += Math.Pow(deltaRx, 2) + Math.Pow(deltaRy, 2) + Math.Pow(deltaRz, 2);

                }

                return Math.Sqrt(sumsq);
            };

            // Numerical gradient calculation
            static Vector<double> NumericalGradient(Func<Vector<double>, double> objectiveFunction, Vector<double> parameters, double epsilon = 1e-8)
            {
                int dim = parameters.Count;
                Vector<double> gradient = Vector<double>.Build.Dense(dim);

                for (int j = 0; j < dim; j++)
                {
                    // Clone and perturb parameter
                    Vector<double> perturbedParams = parameters.Clone();
                    perturbedParams[j] += epsilon;

                    double f1 = objectiveFunction(perturbedParams);
                    double f0 = objectiveFunction(parameters);

                    gradient[j] = (f1 - f0) / epsilon;  // Approximate derivative
                }

                return gradient;
            }


            // Define initial guess (must match 6 parameters)
            var initialGuess = Vector<double>.Build.DenseOfArray(new double[] { boreasBase.rx.D2R(), boreasBase.ry.D2R(), boreasBase.rz.D2R(), boreasTool.rx.D2R(), boreasTool.ry.D2R(), boreasTool.rz.D2R() });

            // Define the optimization problem
            var objective = ObjectiveFunction.Gradient(objectiveFunction, parameters => NumericalGradient(objectiveFunction, parameters));
            var objectiveBaseOnly = ObjectiveFunction.Gradient(objectiveFunctionBaseOnly, parameters => NumericalGradient(objectiveFunctionBaseOnly, parameters));

            var solver = new BfgsMinimizer(1e-8, 1e-8, 10000);

            // Initial best guess from normal optimization
            var bestResult = solver.FindMinimum(objectiveBaseOnly, initialGuess);            

            initialGuess = bestResult.MinimizingPoint;
            bestResult = solver.FindMinimum(objective, initialGuess);

            boreasBase = new cTransform(0, 0, 0, bestResult.MinimizingPoint[0].R2D(), bestResult.MinimizingPoint[1].R2D(), bestResult.MinimizingPoint[2].R2D());
            boreasTool = new cTransform(0, 0, 0, bestResult.MinimizingPoint[3].R2D(), bestResult.MinimizingPoint[4].R2D(), bestResult.MinimizingPoint[5].R2D());

            Console.WriteLine("\nTransformed Boreas w/ Transformation Parameters:");
            var result = testTransform(boreas, measured, boreasBase, boreasTool);

            // Output final best result
            Console.WriteLine("\nFinal Optimized Transformation Parameters:");
            for (int i = 0; i < bestResult.MinimizingPoint.Count; i++)
            {
                Console.WriteLine($"p{i + 1} = {bestResult.MinimizingPoint[i].R2D():F6}");
            }

            Console.WriteLine($"RMS Error = {result.rms:F6}");
            Console.WriteLine($"Average Error = {result.avg:F6}");
            Console.WriteLine($"Standard Deviation = {result.stddev:F6}");
            Console.WriteLine($"A3S = {result.avg + 3 * result.stddev:F6}");

        }


        public static (double rms, double avg, double stddev) testTransform(
            List<cPose> boreas, List<cPose> measured, 
            cTransform boreasBase, cTransform boreasTool)
        {
            double sumsq = 0;
            double sum = 0;
            List<double> errors = new List<double>();

            if (boreas.Count != measured.Count)
                throw new InvalidOperationException("Mismatch between boreas and measured data counts.");

            for (int i = 0; i < boreas.Count; i++)
            {
                // Transform boreasXYZ using xbase and xtool
                cTransform boreasXYZ = (boreasBase.getLHT() * boreas[i].getLHT() * boreasTool.getLHT()).getTransformEulerXYZ();

                Console.WriteLine($"{measured[i].rx:F3},{measured[i].ry:F3},{measured[i].rz:F3} - {boreasXYZ.rx:F3},{boreasXYZ.ry:F3},{boreasXYZ.rz:F3}");

                // Normalize angles
                double normRx = boreasXYZ.rx.m180p180();
                double normRy = boreasXYZ.ry.m180p180();
                double normRz = boreasXYZ.rz.m180p180();

                double deltaRx = (normRx - measured[i].rx).m180p180();
                double deltaRy = (normRy - measured[i].ry).m180p180();
                double deltaRz = (normRz - measured[i].rz).m180p180();

                double error = Math.Sqrt(deltaRx * deltaRx + deltaRy * deltaRy + deltaRz * deltaRz);

                sumsq += error * error;
                sum += error;
                errors.Add(error);
            }

            int n = errors.Count;
            double rms = Math.Sqrt(sumsq);
            double avg = sum / n;
            double stddev = Math.Sqrt(errors.Sum(e => Math.Pow(e - avg, 2)) / n);

            return (rms, avg, stddev);
        }



        public static double[] NelderMead(Func<double[], double> func, double[] start, int maxIterations = 1000)
        {
            int n = start.Length;
            double[][] simplex = new double[n + 1][];

            // Initialize simplex
            for (int i = 0; i < n + 1; i++)
            {
                simplex[i] = (double[])start.Clone();
                if (i > 0)
                    simplex[i][i - 1] += 0.05; // Small shift to generate simplex
            }

            for (int iter = 0; iter < maxIterations; iter++)
            {
                Array.Sort(simplex, (a, b) => func(a).CompareTo(func(b)));

                // Compute centroid
                double[] centroid = new double[n];
                for (int i = 0; i < n; i++)
                    for (int j = 0; j < n; j++)
                        centroid[j] += simplex[i][j] / n;

                // Reflect worst point
                double[] reflected = new double[n];
                for (int j = 0; j < n; j++)
                    reflected[j] = centroid[j] + (centroid[j] - simplex[n][j]);

                if (func(reflected) < func(simplex[n - 1]))
                    simplex[n] = reflected;
            }

            return simplex[0]; // Best solution
        }
    }

}
