using Accord.Math;
using Accord.Math.Optimization;
using ArtemisCore.Calculations;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Optimization;
using MathNet.Numerics.Optimization.TrustRegion;
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
        public static void Solve()
        {
            List<cPose> boreas = new List<cPose>();
            List<cPose> measured = new List<cPose>();


            boreas.Add(new cPose(0, 0, 0, -2.92, 6.302, -153.648));
            boreas.Add(new cPose(0, 0, 0, -7.468, 4.084, -153.292));
            boreas.Add(new cPose(0, 0, 0, -7.4, 4.12, -153.307));
            boreas.Add(new cPose(0, 0, 0, -7.374, 4.122, -153.298));
            boreas.Add(new cPose(0, 0, 0, -0.779, -9.465, -153.389));
            boreas.Add(new cPose(0, 0, 0, -0.836, -9.495, -145.708));
            boreas.Add(new cPose(0, 0, 0, 1.928, -7.593, -145.223));
            boreas.Add(new cPose(0, 0, 0, -6.405, -13.205, -146.814));
            boreas.Add(new cPose(0, 0, 0, -6.524, -13.25, -146.676));
            boreas.Add(new cPose(0, 0, 0, 15.876, 2.412, -144.276));
            boreas.Add(new cPose(0, 0, 0, 15.894, 2.412, -144.144));
            boreas.Add(new cPose(0, 0, 0, 4.67, 17.276, -142.1));
            boreas.Add(new cPose(0, 0, 0, 4.699, 17.432, -158.683));
            boreas.Add(new cPose(0, 0, 0, 11.151, 0.796, -159.626));
            boreas.Add(new cPose(0, 0, 0, 7.143, 22.63, -168.598));
            boreas.Add(new cPose(0, 0, 0, 7.127, 22.615, -168.594));
            boreas.Add(new cPose(0, 0, 0, 12.086, -2.845, -169.471));
            boreas.Add(new cPose(0, 0, 0, 11.676, -3.224, -134.397));
            boreas.Add(new cPose(0, 0, 0, -2.765, 10.85, -133.502));
            boreas.Add(new cPose(0, 0, 0, 0.286, -0.066, -152.216));
            boreas.Add(new cPose(0, 0, 0, 0.266, -0.052, -152.199));
            boreas.Add(new cPose(0, 0, 0, 0.254, 0.029, -143.109));
            boreas.Add(new cPose(0, 0, 0, 15.358, 11.183, -144.667));
            boreas.Add(new cPose(0, 0, 0, 29.075, -8.335, -144.905));
            boreas.Add(new cPose(0, 0, 0, -25.882, 47.517, -122.895));
            boreas.Add(new cPose(0, 0, 0, 15.62, 10.772, -150.114));
            boreas.Add(new cPose(0, 0, 0, 7.152, 5.785, -148.888));


            measured.Add(new cPose(0, 0, 0, -0.401, 6.916, 1.786));
            measured.Add(new cPose(0, 0, 0, 4.677, 7.095, 1.222));
            measured.Add(new cPose(0, 0, 0, 4.597, 7.094, 1.242));
            measured.Add(new cPose(0, 0, 0, 4.58, 7.085, 1.226));
            measured.Add(new cPose(0, 0, 0, 5.143, -7.996, 1.639));
            measured.Add(new cPose(0, 0, 0, 5.199, -7.998, -6.048));
            measured.Add(new cPose(0, 0, 0, 1.864, -7.619, -6.566));
            measured.Add(new cPose(0, 0, 0, 11.837, -8.689, -5.073));
            measured.Add(new cPose(0, 0, 0, 11.964, -8.666, -5.22));
            measured.Add(new cPose(0, 0, 0, -15.212, -5.242, -8.795));
            measured.Add(new cPose(0, 0, 0, -15.232, -5.251, -8.932));
            measured.Add(new cPose(0, 0, 0, -12.345, 13.031, -9.235));
            measured.Add(new cPose(0, 0, 0, -12.435, 13.161, 7.357));
            measured.Add(new cPose(0, 0, 0, -10.247, -4.514, 7.106));
            measured.Add(new cPose(0, 0, 0, -17.107, 16.608, 17.635));
            measured.Add(new cPose(0, 0, 0, -17.09, 16.6, 17.635));
            measured.Add(new cPose(0, 0, 0, -9.387, -8.169, 17.059));
            measured.Add(new cPose(0, 0, 0, -8.862, -8.325, -17.964));
            measured.Add(new cPose(0, 0, 0, -2.731, 10.845, -18.023));
            measured.Add(new cPose(0, 0, 0, -0.231, -0.207, 0.169));
            measured.Add(new cPose(0, 0, 0, -0.221, -0.188, 0.153));
            measured.Add(new cPose(0, 0, 0, -0.252, -0.11, -8.937));
            measured.Add(new cPose(0, 0, 0, -18.725, 2.808, -8.423));
            measured.Add(new cPose(0, 0, 0, -22.42, -20.726, -9.113));
            measured.Add(new cPose(0, 0, 0, -8.035, 52.129, -13.675));
            measured.Add(new cPose(0, 0, 0, -18.763, 2.327, -3.022));
            measured.Add(new cPose(0, 0, 0, -9.031, 1.754, -3.377));




            // Objective function
            Func<Vector<double>, double> objectiveFunction = (Vector<double> x) =>
            {
                double sum = 0;

                // Initialize transforms
                cTransform xbase = new cTransform(0, 0, 0, x[0], x[1], x[2]);
                cTransform xtool = new cTransform(0, 0, 0, x[3], x[4], x[5]);

                // Ensure lists match in size
                if (boreas.Count != measured.Count)
                    throw new InvalidOperationException("Mismatch between boreas and measured data counts.");

                // Compute error sum
                for (int i = 0; i < boreas.Count; i++)
                {
                    // Transform boreasXYZ using xbase and xtool
                    cTransform boreasXYZ = xbase * boreas[i].getTransform() * xtool;

                    // Normalize angles to [-180, 180] before subtraction
                    double normRx = boreasXYZ.rx.m180p180();
                    double normRy = boreasXYZ.ry.m180p180();
                    double normRz = boreasXYZ.rz.m180p180();

                    double deltaRx = (normRx - measured[i].rx).m180p180();
                    double deltaRy = (normRy - measured[i].ry).m180p180();
                    double deltaRz = (normRz - measured[i].rz).m180p180();

                    // Sum of squared errors
                    sum += Math.Pow(deltaRx, 2) + Math.Pow(deltaRy, 2) + Math.Pow(deltaRz, 2);

                }

                return Math.Sqrt(sum);
            };

            // Numerical gradient calculation
            static Vector<double> NumericalGradient(Func<Vector<double>, double> objectiveFunction, Vector<double> parameters, double epsilon = 1e-6)
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
            cTransform boreasBase = new cTransform(0, 0, 0, 180, 0, 152);
            cTransform boreasTool = new cTransform(0, 0, 0, 180, 0, 0);

            //double testOutput = testTransform(boreas, measured, boreasBase, boreasTool);

            double[] initialGuessN = { 0, 0, 0, 0, 0, 0 };
            double[] bestParams = NelderMead(parameters => objectiveFunction(Vector<double>.Build.DenseOfArray(parameters)), initialGuessN);


            // Define initial guess (must match 6 parameters)
            var initialGuess = Vector<double>.Build.DenseOfArray(new double[] { boreasBase.rx, boreasBase.ry, boreasBase.rz, boreasTool.rx, boreasTool.ry, boreasTool.rz });

            // Define the optimization problem
            var objective = ObjectiveFunction.Gradient(
                objectiveFunction,
                parameters => NumericalGradient(objectiveFunction, parameters)
            );

            var solver = new BfgsMinimizer(1e-5, 1e-5, 1000);

            var random = new Random();

            // Initial best guess from normal optimization
            var bestResult = solver.FindMinimum(objective, initialGuess);
            var bestError = bestResult.FunctionInfoAtMinimum.Value;

            Console.WriteLine($"Initial best error: {bestError:F6}");

            for (int i = 0; i < 10; i++) // Try 10 perturbations
            {
                Vector<double> perturbedGuess;

                // Every 5th iteration, reset completely to a random starting point
                if (i % 5 == 0)
                {
                    perturbedGuess = Vector<double>.Build.DenseOfArray(new double[]
                    {
                        (random.NextDouble() - 0.5) * 10,  // Larger random range
                        (random.NextDouble() - 0.5) * 10,
                        (random.NextDouble() - 0.5) * 10,
                        (random.NextDouble() - 0.5) * 10,
                        (random.NextDouble() - 0.5) * 10,
                        (random.NextDouble() - 0.5) * 10
                    });

                    Console.WriteLine($"Iteration {i}: Full restart with random guess.");
                }
                else
                {
                    // Small perturbation around the best known solution
                    perturbedGuess = bestResult.MinimizingPoint
                        .Map(v => v + (random.NextDouble() - 0.5) * 0.1); // Increase perturbation range
                }

                // Solve again from the new start point
                var newResult = solver.FindMinimum(objective, perturbedGuess);
                var newError = newResult.FunctionInfoAtMinimum.Value;

                Console.WriteLine($"Iteration {i}: New error = {newError:F6}");

                // Keep the best solution
                if (newError < bestError)
                {
                    bestResult = newResult;
                    bestError = newError;
                    Console.WriteLine($"Iteration {i}: Found new best solution with error {bestError:F6}");
                }
            }

            boreasBase = new cTransform(0, 0, 0, bestResult.MinimizingPoint[0], bestResult.MinimizingPoint[1], bestResult.MinimizingPoint[2]);
            boreasTool = new cTransform(0, 0, 0, bestResult.MinimizingPoint[3], bestResult.MinimizingPoint[4], bestResult.MinimizingPoint[5]);

            testTransform(boreas, measured, boreasBase, boreasTool);

            // Output final best result
            Console.WriteLine("\nFinal Optimized Transformation Parameters:");
            for (int i = 0; i < bestResult.MinimizingPoint.Count; i++)
            {
                Console.WriteLine($"p{i + 1} = {bestResult.MinimizingPoint[i]:F6}");
            }

            Console.WriteLine($"Best Function Value = {bestResult.FunctionInfoAtMinimum.Value:F6}");

        }


        public static double testTransform(List<cPose> boreas, List<cPose> measured, cTransform boreasBase, cTransform boreasTool)
        {
            double sum = 0;

            // Ensure lists match in size
            if (boreas.Count != measured.Count)
                throw new InvalidOperationException("Mismatch between boreas and measured data counts.");

            // Compute error sum
            for (int i = 0; i < boreas.Count; i++)
            {
                // Transform boreasXYZ using xbase and xtool
                cTransform boreasXYZ = (boreasBase.getLHT() * boreas[i].getLHT() * boreasTool.getLHT()).getTransformEulerXYZ();

                Console.WriteLine($"{boreasXYZ.rx:F3},{boreasXYZ.ry:F3},{boreasXYZ.rz:F3}");

                // Normalize angles to [-180, 180] before subtraction
                double normRx = boreasXYZ.rx.m180p180();
                double normRy = boreasXYZ.ry.m180p180();
                double normRz = boreasXYZ.rz.m180p180();

                double deltaRx = (normRx - measured[i].rx).m180p180();
                double deltaRy = (normRy - measured[i].ry).m180p180();
                double deltaRz = (normRz - measured[i].rz).m180p180();

                // Sum of squared errors
                sum += Math.Sqrt(Math.Pow(deltaRx, 2) + Math.Pow(deltaRy, 2) + Math.Pow(deltaRz, 2));

            }
            return Math.Sqrt(sum);
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
