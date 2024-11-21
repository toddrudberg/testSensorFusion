using ArtemisCore.Calculations;
using System;
using System.Runtime.InteropServices;
using System.Windows.Forms;

namespace Test_Sensor_Fusion_Issues
{
    public partial class Form1 : Form
    {
        // Import the AllocConsole method from kernel32.dll
        [DllImport("kernel32.dll")]
        private static extern bool AllocConsole();

        // Import the FreeConsole method to close the console
        [DllImport("kernel32.dll")]
        private static extern bool FreeConsole();

        // Import the SetWindowPos method to move the console window
        [DllImport("user32.dll", SetLastError = true)]
        private static extern bool SetWindowPos(IntPtr hWnd, IntPtr hWndInsertAfter, int X, int Y, int cx, int cy, uint uFlags);

        // Get the handle of the console window
        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern IntPtr GetConsoleWindow();

        public Form1()
        {
            InitializeComponent();
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            // Launch and position the console
            LaunchConsole();
            PositionConsole();

            // Position the form in the upper-right quarter
            PositionForm();


            //test normalizeOrientation
            cTransform t = new cTransform(0, 0, 0, 0, 5, -5, 5);
            cTransform result = t.getLHT().normalizeOrientation().getTransformEulerXYZ();
            cTransform result2 = t.getLHT().getTransformEulerXYZ();

            cLHT test = new cLHT();
            test.M = cLHT.ensureOrthonormality(t.getLHT());
            cTransform result3 = test.getTransformEulerXYZ();


            Console.WriteLine();
            Console.WriteLine("Original: " + t.getPose().ToString());
            Console.WriteLine("Result: " + result.getPose().ToString());
            Console.WriteLine("Result2: " + result2.getPose().ToString());
            Console.WriteLine("Result3: " + result3.getPose().ToString());
            Console.WriteLine();


            Random random = new Random();

            double OrientationErrorMagnitude = 2.0;
            double linearErrorMagnitude = 5.0;


            cPose Target1 = new cPose(0,0,0,0,0,0);
            cPose Target2 = new cPose(5,0,0,0,0,0);
            cPose Measured = new cPose(
                    random.NextDouble() * linearErrorMagnitude + (Target1.X + Target2.X) / 2,
                    random.NextDouble() * linearErrorMagnitude + (Target1.Y + Target2.Y) / 2,
                    random.NextDouble() * linearErrorMagnitude + (Target1.Z + Target2.Z) / 2,
                    random.NextDouble() * OrientationErrorMagnitude - OrientationErrorMagnitude / 2 + (Target1.rX + Target2.rX) / 2,
                    random.NextDouble() * OrientationErrorMagnitude - OrientationErrorMagnitude / 2 + (Target1.rY + Target2.rY) / 2,
                    random.NextDouble() * OrientationErrorMagnitude - OrientationErrorMagnitude / 2 + (Target1.rZ + Target2.rZ) / 2
                );

            cPose TPError = new cPose(0, 0, 0, 0, 0, 0);

            cXYZ lastXYZGoal = new cXYZ(Target1.X, Target1.Y, Target1.Z);
            cXYZ xyzGoal = new cXYZ(Target2.X, Target2.Y, Target2.Z);

            cXYZ xyzMetrology = new cXYZ(Measured.x, Measured.y, Measured.z);
            cXYZ xyzDeltaGoal = xyzGoal - lastXYZGoal;

            cXYZ xyzVectorGoalMetrology = xyzMetrology - lastXYZGoal;
            cXYZ xyzDeltaGoalUnitVector = xyzDeltaGoal * (1.0 / xyzDeltaGoal.getMagnitude);
            cXYZ xParallel = xyzDeltaGoalUnitVector * (xyzDeltaGoalUnitVector * xyzVectorGoalMetrology);
            cXYZ xPerpendicular = xyzVectorGoalMetrology - xParallel;

            cLHT errorLHT = (Target2.getLHT() * !Measured.getLHT());
            cLHT targetPrime = (Target2.getLHT() * errorLHT).normalizeOrientation();

            targetPrime.M[0, 3] = Target2.X - xPerpendicular.X;
            targetPrime.M[1, 3] = Target2.Y - xPerpendicular.Y;
            targetPrime.M[2, 3] = Target2.Z - xPerpendicular.Z;

            cPose newTarget = targetPrime.getPoseEulerXYZ();

            cPose newTargetOldWay = new cPose(newTarget.x, newTarget.y, newTarget.z, Target2.rx - Measured.rx, Target2.ry - Measured.ry, Target2.rz - Measured.rz);

            Console.WriteLine("Target1: " + Target1.ToString());
            Console.WriteLine("Target2: " + Target2.ToString());
            Console.WriteLine("Measured: " + Measured.ToString());
            Console.WriteLine("TPError: " + TPError.ToString());
            Console.WriteLine("New Target: " + newTarget.ToString());
            Console.WriteLine("New Target Old " + newTargetOldWay.ToString());
        }

        private void LaunchConsole()
        {
            AllocConsole();
            Console.WriteLine("Console launched!");
            Console.WriteLine("You can use this console to debug or log data.");
        }

        private void PositionConsole()
        {
            IntPtr consoleHandle = GetConsoleWindow();
            if (consoleHandle == IntPtr.Zero)
                return;

            // Get the screen dimensions
            int screenWidth = Screen.PrimaryScreen.Bounds.Width;
            int screenHeight = Screen.PrimaryScreen.Bounds.Height;

            // Calculate the size and position for the upper-left quarter
            int width = screenWidth / 2;
            int height = screenHeight / 2;
            int x = 0;
            int y = 0;

            // Set the console window position and size
            SetWindowPos(consoleHandle, IntPtr.Zero, x, y, width, height, 0);
        }

        private void PositionForm()
        {
            // Get the screen dimensions
            int screenWidth = Screen.PrimaryScreen.Bounds.Width;
            int screenHeight = Screen.PrimaryScreen.Bounds.Height;

            // Calculate the size and position for the upper-right quarter
            int width = screenWidth / 2;
            int height = screenHeight / 2;
            int x = screenWidth / 2;
            int y = 0;

            // Set the form size and location
            this.Size = new System.Drawing.Size(width, height);
            this.Location = new System.Drawing.Point(x, y);
        }

        private void CloseConsole()
        {
            FreeConsole();
        }
    }
}
