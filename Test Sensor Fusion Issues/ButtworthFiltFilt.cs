using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Test_Sensor_Fusion_Issues
{


    using System;

    public class ButterworthFilter
    {
        private readonly double[] a; // Feedback coefficients
        private readonly double[] b; // Feedforward coefficients
        private readonly double[] x; // Input history (circular buffer)
        private readonly double[] y; // Output history (circular buffer)
        private int bufferIndex;     // Circular buffer index

        public ButterworthFilter()
        {
b = new double[] {
    0.00000754,
    0.00003769,
    0.00007538,
    0.00007538,
    0.00003769,
    0.00000754,
};
a = new double[] {
    1.00000000,
    -4.34966577,
    7.60513781,
    -6.67804519,
    2.94381226,
    -0.52099790,
};

            int order = a.Length - 1; // Order of the filter
            x = new double[order + 1];
            y = new double[order + 1];
            bufferIndex = 0;
        }

        // Filters one sample
        public double FilterSingle(double input)
        {
            bufferIndex = (bufferIndex + 1) % x.Length; // Update circular buffer index
            x[bufferIndex] = input; // Store current input

            // Compute the filtered value
            y[bufferIndex] = b[0] * x[bufferIndex];
            for (int i = 1; i < b.Length; i++)
            {
                int idx = (bufferIndex - i + x.Length) % x.Length;
                y[bufferIndex] += b[i] * x[idx];
            }
            for (int i = 1; i < a.Length; i++)
            {
                int idx = (bufferIndex - i + y.Length) % y.Length;
                y[bufferIndex] -= a[i] * y[idx];
            }

            return y[bufferIndex];
        }

        // Filters a batch of data
        public double[] FilterBatch(double[] inputSignal)
        {
            double[] outputSignal = new double[inputSignal.Length];

            for (int i = 0; i < inputSignal.Length; i++)
            {
                outputSignal[i] = FilterSingle(inputSignal[i]);
            }

            return outputSignal;


        }

        public double[] FilterForwardReverse(double[] inputSignal)
        {
            int n = inputSignal.Length;
            double[] forwardFiltered = new double[n];
            double[] reversedSignal = new double[n];
            double[] backwardFiltered = new double[n];

            // Forward filter
            for (int i = 0; i < n; i++)
            {
                forwardFiltered[i] = FilterSingle(inputSignal[i]);
            }

            // Reverse the forward-filtered signal
            for (int i = 0; i < n; i++)
            {
                reversedSignal[i] = forwardFiltered[n - 1 - i];
            }

            // Reset the filter state (optional, ensures clean backward pass)
            ResetFilterState();

            // Backward filter
            for (int i = 0; i < n; i++)
            {
                backwardFiltered[i] = FilterSingle(reversedSignal[i]);
            }

            // Reverse again to restore original order
            double[] finalFiltered = new double[n];
            for (int i = 0; i < n; i++)
            {
                finalFiltered[i] = backwardFiltered[n - 1 - i];
            }

            return finalFiltered;
        }

        // public double FilterForwardReverse1Output(double[] inputSignal)
        // {
        //     int n = inputSignal.Length;
        //     double[] forwardFiltered = new double[n];
        //     double[] reversedSignal = new double[n];
        //     double[] backwardFiltered = new double[n];

        //     // Forward filter
        //     for (int i = 0; i < n; i++)
        //     {
        //         forwardFiltered[i] = FilterSingle(inputSignal[i]);
        //     }

        //     // Reverse the forward-filtered signal
        //     for (int i = 0; i < n; i++)
        //     {
        //         reversedSignal[i] = forwardFiltered[n - 1 - i];
        //     }

        //     // Reset the filter state (optional, ensures clean backward pass)
        //     ResetFilterState();

        //     // Backward filter
        //     for (int i = 0; i < n; i++)
        //     {
        //         backwardFiltered[i] = FilterSingle(reversedSignal[i]);
        //     }

        //     // Reverse again to restore original order
        //     double[] finalFiltered = new double[n];
        //     for (int i = 0; i < n; i++)
        //     {
        //         finalFiltered[i] = backwardFiltered[n - 1 - i];
        //     }

        //     //return only the last
        //     return finalFiltered[finalFiltered.Length - 1];
        // }


        public double FilterForwardReverse1Output(double[] inputSignal)
        {
            int n = inputSignal.Length;

            // Forward filter
            double[] forwardFiltered = new double[n];
            for (int i = 0; i < n; i++)
            {
                forwardFiltered[i] = FilterSingle(inputSignal[i]);
            }

            // Reverse the forward-filtered signal
            double[] reversedSignal = new double[n];
            for (int i = 0; i < n; i++)
            {
                reversedSignal[i] = forwardFiltered[n - 1 - i];
            }

            // Reset the filter state
            ResetFilterState();

            // Backward filter
            double[] backwardFiltered = new double[n];
            for (int i = 0; i < n; i++)
            {
                backwardFiltered[i] = FilterSingle(reversedSignal[i]);
            }

            // Reverse again to restore original order
            double[] finalFiltered = new double[n];
            for (int i = 0; i < n; i++)
            {
                finalFiltered[i] = backwardFiltered[n - 1 - i];
            }

            // Return only the last value
            return finalFiltered[finalFiltered.Length - 1];
        }



        // Optional: Reset filter state (e.g., clear history buffers)
        public void ResetFilterState()
        {
            Array.Clear(x, 0, x.Length);
            Array.Clear(y, 0, y.Length);
        }        

        public static double[] GenerateTestSignal(int length)
        {
            double[] signal = new double[length];
            Random random = new Random();

            double dataRate = 1000 / 4; // 250Hz
            double frequency = .3;
            // Generate a random frequency between 80 and 120 Hz

            for(int i = 0; i < length; i++)
            {
                double randomFrequency = 80 + 40 * random.NextDouble(); // 80 + (0 to 40)

                // Generate the signal
                signal[i] = Math.Sin(2 * Math.PI * frequency * i / dataRate) +      // 2 Hz base signal
                            0.5 * (1 - random.NextDouble()) * Math.Sin(2 * Math.PI * 8 * i / dataRate) +  // 8 Hz noise
                            0.2 * (1 - random.NextDouble()) * Math.Sin(2 * Math.PI * 16 * i / dataRate) + // 16 Hz noise
                            0.2 * (1 - random.NextDouble()) * Math.Sin(2 * Math.PI * randomFrequency * i / dataRate); // Variable noise                        
            }

            return signal;
        }
    }
}

