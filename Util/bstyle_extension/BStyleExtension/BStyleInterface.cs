using System;
using System.Diagnostics;
using System.IO;
using System.Runtime.InteropServices;
using System.Windows.Forms;

namespace BStyleExtension
{
    public class BStyleInterface
    {
        public delegate void BStyleErrorHandler(object sender, BStyleErrorArgs e);
        public event BStyleErrorHandler ErrorRaised;

        private BStyleErrorDelegate _BStyleError;
        private BStyleMemAllocDelegate _BStyleMemAlloc;

        private delegate void BStyleErrorDelegate(int errorNum, [MarshalAs(UnmanagedType.LPStr)] String error);
        private delegate IntPtr BStyleMemAllocDelegate(int size);

        public BStyleInterface()
        {
            _BStyleMemAlloc = OnBStyleMemAlloc;
            _BStyleError = OnBStyleError;
        }

        /// Call the BStyle-Wrapper inside WSL to format the given source file.
        /// Returns textIn on error, formatted text on success.
        public String FormatSourceFile(String sourceFilePath, String textIn, String bStylePath, String bashPath, bool keepTempFiles)
        {
            string outFilePath = sourceFilePath + ".bstyle-tmp";

            if (String.IsNullOrEmpty(sourceFilePath) || !File.Exists(sourceFilePath))
            {
                MessageBox.Show(String.Format("The source file '{0}' cannot be found.", sourceFilePath));
                return textIn;
            }
            if (String.IsNullOrEmpty(bStylePath) || !File.Exists(bStylePath))
            {
                MessageBox.Show(String.Format("The given B-Style path \"{0}\" cannot be found.", bStylePath));
                return textIn;
            }
            if (String.IsNullOrEmpty(bashPath) || !File.Exists(bashPath))
            {
                MessageBox.Show(String.Format("The given bash.exe path \"{0}\" cannot be found.", bashPath));
                return textIn;
            }

            // The path to BStyle must be absolute when using WSL.
            if (bStylePath.Length < 3 || !Char.IsLetter(bStylePath[0]) || bStylePath[1] != ':' || bStylePath[2] != '\\')
            {
                MessageBox.Show(String.Format("The given B-Style path \"{0}\" is not absolute.", bStylePath));
                return textIn;
            }

            if (textIn.Length != 0)
            {
                Process bstyle = new Process
                {
                    StartInfo = new ProcessStartInfo
                    {
                        FileName = bashPath,
                        // bstyle-extension reads the source file and creates a temp file with the bstyle output
                        Arguments = String.Format("-c \"./bstyle-extension '{0}'\"", sourceFilePath),
                        WorkingDirectory = Path.GetDirectoryName(bStylePath),
                        UseShellExecute = false,
                        RedirectStandardInput = true,
                        RedirectStandardOutput = true,
#if DEBUG
                        RedirectStandardError = true,
#endif
                        CreateNoWindow = true,
                    }
                };

                bstyle.Start();

#if DEBUG
                String err = bstyle.StandardError.ReadToEnd();
                if (!String.IsNullOrEmpty(err))
                {
                    MessageBox.Show(String.Format("Calling B-Style returned an error:\n{0}", err), "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                }
#endif
                bstyle.WaitForExit();
                bstyle.Close();

                // Read the formatted file back
                try
                {
                    string textOut = File.ReadAllText(outFilePath, System.Text.Encoding.UTF8);

                    if (!keepTempFiles)
                    {
                        File.Delete(outFilePath);
                    }

                    return textOut;
                } catch (Exception ex)
                {
                    MessageBox.Show(String.Format("Reading the B-Style output failed:\n{0}", ex.ToString()));
                }
            }

            return textIn;
        }

        /// Calls BStyle directly inside WSL to format textIn.
        /// Returns textIn on error, formatted text on success.
        public String FormatSource(String textIn, String bStylePath, String bashPath)
        {
            if (String.IsNullOrEmpty(bStylePath) || !File.Exists(bStylePath))
            {
                MessageBox.Show(String.Format("The given B-Style path \"{0}\" cannot be found.", bStylePath));
                return textIn;
            }
            if (String.IsNullOrEmpty(bashPath) || !File.Exists(bashPath))
            {
                MessageBox.Show(String.Format("The given bash.exe path \"{0}\" cannot be found.", bashPath));
                return textIn;
            }

            // Return the allocated string
            String textOut = String.Empty;
            if (textIn.Length != 0)
            {
                // The path to BStyle must be absolute when using WSL.
                // TODO: upgrade to >= .NET 5 and use Path.IsPathFullyQualified here.
                if (bStylePath.Length < 3 || !Char.IsLetter(bStylePath[0]) || bStylePath[1] != ':' || bStylePath[2] != '\\')
                {
                    MessageBox.Show(String.Format("The given B-Style path \"{0}\" is not absolute.", bStylePath));
                    return textIn;
                }

                Process bstyle = new Process
                {
                    StartInfo = new ProcessStartInfo
                    {
                        FileName = bashPath, // e.g "C:\Windows\System32\bash.exe"
                        Arguments = Path.GetFileName(bStylePath), // usually just "bstyle"
                        WorkingDirectory = Path.GetDirectoryName(bStylePath),   // e.g. "C:\path\to\B-Human\Make\Common\"
                        UseShellExecute = false,
                        RedirectStandardInput = true,
                        RedirectStandardOutput = true,
#if DEBUG
                        RedirectStandardError = true,
#endif
                        CreateNoWindow = true,
                    }
                };

                bstyle.Start();

                bstyle.StandardInput.Write(textIn);
                bstyle.StandardInput.Close();

                textOut = bstyle.StandardOutput.ReadToEnd();
#if DEBUG
                String err = bstyle.StandardError.ReadToEnd();
                if (!String.IsNullOrEmpty(err))
                {
                    MessageBox.Show(String.Format("Calling B-Style returned an error:\n{0}", err), "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                }
#endif
                bstyle.WaitForExit();
                bstyle.Close();
            }
#if DEBUG
            MessageBox.Show(String.Format("Input:\n{0}\n\nOutput:\n{1}", textIn, textOut));
#endif
            return textOut;
        }

        // Allocate the memory for the Artistic Style return string.
        private IntPtr OnBStyleMemAlloc(int size)
        {
            return Marshal.AllocHGlobal(size);
        }

        private void OnBStyleError(object source, BStyleErrorArgs args)
        {
            ErrorRaised?.Invoke(source, args);
        }

        private void OnBStyleError(int errorNumber, String errorMessage)
        {
            OnBStyleError(this, new BStyleErrorArgs(errorNumber + ": " + errorMessage));
        }
    }
}