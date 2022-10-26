using System;
using System.ComponentModel;
using System.Runtime.InteropServices;
using System.Windows.Forms;
using Microsoft.VisualStudio.Shell;

namespace BStyleExtension
{
    [CLSCompliant(false), ComVisible(true)]
    public class BStyleGeneralOptionsPage : DialogPage
    {
        private BStyleGeneralOptionsControl _control;

        public bool CppFormatOnSave { get; set; }
        public bool CppKeepTempFiles { get; set; }
        public String CppBStylePath { get; set; }
        public String CppBashPath { get; set; }

        [Browsable(false), DesignerSerializationVisibility(DesignerSerializationVisibility.Hidden)]
        protected override IWin32Window Window
        {
            get
            {
                _control = new BStyleGeneralOptionsControl
                {
                    CppFormatOnSave = CppFormatOnSave,
                    CppKeepTempFiles = CppKeepTempFiles,
                    CppBStylePath = CppBStylePath,
                    CppBashPath = CppBashPath
                };

                return _control;
            }
        }

        protected override void OnDeactivate(CancelEventArgs e)
        {
            if (_control != null)
            {
                CppFormatOnSave = _control.CppFormatOnSave;
                CppKeepTempFiles = _control.CppKeepTempFiles;
                CppBStylePath = _control.CppBStylePath;
                CppBashPath = _control.CppBashPath;
            }

            base.OnDeactivate(e);
        }

        protected override void OnActivate(CancelEventArgs e)
        {
            if (_control != null)
            {
                _control.CppFormatOnSave = CppFormatOnSave;
                _control.CppKeepTempFiles = CppKeepTempFiles;
                _control.CppBStylePath = CppBStylePath;
                _control.CppBashPath = CppBashPath;
            }

            base.OnActivate(e);
        }

        protected override void OnApply(PageApplyEventArgs e)
        {
            if (_control != null)
            {
                CppFormatOnSave = _control.CppFormatOnSave;
                CppKeepTempFiles = _control.CppKeepTempFiles;
                CppBStylePath = _control.CppBStylePath;
                CppBashPath = _control.CppBashPath;
            }

            base.OnApply(e);
        }
    }
}
