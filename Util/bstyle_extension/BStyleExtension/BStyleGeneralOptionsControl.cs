using System;
using System.IO;
using System.Windows.Forms;
using System.Xml.Serialization;

namespace BStyleExtension {
    public partial class BStyleGeneralOptionsControl : UserControl
    {
        private bool _cppFormatOnSave;
        private bool _cppKeepTempFiles;
        private String _cppBStylePath;
        private String _cppBashPath;

        public bool CppFormatOnSave
        {
            get
            {
                _cppFormatOnSave = checkBoxCppFormatOnSave.Checked;
                return _cppFormatOnSave;
            }
            set
            {
                _cppFormatOnSave = value;
                checkBoxCppFormatOnSave.Checked = value;
            }
        }

        public bool CppKeepTempFiles
        {
            get
            {
                _cppKeepTempFiles = checkBoxCppKeepTempFiles.Checked;
                return _cppKeepTempFiles;
            }
            set
            {
                _cppKeepTempFiles = value;
                checkBoxCppKeepTempFiles.Checked = value;
            }
        }

        public String CppBStylePath
        {
            get
            {
                _cppBStylePath = txtbBStylePath.Text;
                return _cppBStylePath;
            }
            set
            {
                _cppBStylePath = value;
                txtbBStylePath.Text = value;
            }
        }

        public String CppBashPath
        {
            get
            {
                _cppBashPath = txtbBashPath.Text;
                return _cppBashPath;
            }
            set
            {
                _cppBashPath = value;
                txtbBashPath.Text = value;
            }
        }

        public BStyleGeneralOptionsControl()
        {
            InitializeComponent();
        }

        private void btnBStylePath_Click(object sender, EventArgs e)
        {
            if(dlgBStyle.ShowDialog() == DialogResult.OK)
            {
                txtbBStylePath.Text = dlgBStyle.FileName;
            }
        }

        private void btnBashPath_Click(object sender, EventArgs e)
        {
            if(dlgBash.ShowDialog() == DialogResult.OK)
            {
                txtbBashPath.Text = dlgBash.FileName;
            }
        }

        private void txtbBStylePath_TextChanged(object sender, EventArgs e)
        {
            if(File.Exists(txtbBStylePath.Text))
            {
                errorProvider.SetError(lblBStylePath, null);
            }
            else
            {
                errorProvider.SetError(lblBStylePath, "File doesn't seem to exist.");
            }
        }

        private void txtbBashPath_TextChanged(object sender, EventArgs e)
        {
            if(File.Exists(txtbBashPath.Text))
            {
                errorProvider.SetError(lblBashPath, null);
            }
            else
            {
                errorProvider.SetError(lblBashPath, "File doesn't seem to exist.");
            }
        }
    }
}