namespace BStyleExtension
{
    partial class BStyleGeneralOptionsControl
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Component Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.tabPageCPP = new System.Windows.Forms.TabPage();
            this.checkBoxCppKeepTempFiles = new System.Windows.Forms.CheckBox();
            this.btnBashPath = new System.Windows.Forms.Button();
            this.txtbBashPath = new System.Windows.Forms.TextBox();
            this.lblBashPath = new System.Windows.Forms.Label();
            this.btnBStylePath = new System.Windows.Forms.Button();
            this.txtbBStylePath = new System.Windows.Forms.TextBox();
            this.lblBStylePath = new System.Windows.Forms.Label();
            this.checkBoxCppFormatOnSave = new System.Windows.Forms.CheckBox();
            this.tabControlOptions = new System.Windows.Forms.TabControl();
            this.dlgBStyle = new System.Windows.Forms.OpenFileDialog();
            this.dlgBash = new System.Windows.Forms.OpenFileDialog();
            this.errorProvider = new System.Windows.Forms.ErrorProvider(this.components);
            this.tabPageCPP.SuspendLayout();
            this.tabControlOptions.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.errorProvider)).BeginInit();
            this.SuspendLayout();
            // 
            // tabPageCPP
            // 
            this.tabPageCPP.Controls.Add(this.checkBoxCppKeepTempFiles);
            this.tabPageCPP.Controls.Add(this.btnBashPath);
            this.tabPageCPP.Controls.Add(this.txtbBashPath);
            this.tabPageCPP.Controls.Add(this.lblBashPath);
            this.tabPageCPP.Controls.Add(this.btnBStylePath);
            this.tabPageCPP.Controls.Add(this.txtbBStylePath);
            this.tabPageCPP.Controls.Add(this.lblBStylePath);
            this.tabPageCPP.Controls.Add(this.checkBoxCppFormatOnSave);
            this.tabPageCPP.Location = new System.Drawing.Point(4, 22);
            this.tabPageCPP.Name = "tabPageCPP";
            this.tabPageCPP.Padding = new System.Windows.Forms.Padding(3);
            this.tabPageCPP.Size = new System.Drawing.Size(386, 259);
            this.tabPageCPP.TabIndex = 0;
            this.tabPageCPP.Text = "C/C++";
            this.tabPageCPP.UseVisualStyleBackColor = true;
            // 
            // checkBoxCppKeepTempFiles
            //
            this.checkBoxCppKeepTempFiles.AutoSize = true;
            this.checkBoxCppKeepTempFiles.Location = new System.Drawing.Point(261, 3);
            this.checkBoxCppKeepTempFiles.Name = "checkBoxCppKeepTempFiles";
            this.checkBoxCppKeepTempFiles.Size = new System.Drawing.Size(121, 17);
            this.checkBoxCppKeepTempFiles.TabIndex = 13;
            this.checkBoxCppKeepTempFiles.Text = "Keep temporary files";
            this.checkBoxCppKeepTempFiles.UseVisualStyleBackColor = true;
            //
            // btnBashPath
            // 
            this.btnBashPath.Location = new System.Drawing.Point(347, 94);
            this.btnBashPath.Margin = new System.Windows.Forms.Padding(2);
            this.btnBashPath.Name = "btnBashPath";
            this.btnBashPath.Size = new System.Drawing.Size(35, 19);
            this.btnBashPath.TabIndex = 12;
            this.btnBashPath.Text = "...";
            this.btnBashPath.UseVisualStyleBackColor = true;
            this.btnBashPath.Click += new System.EventHandler(this.btnBashPath_Click);
            // 
            // txtbBashPath
            // 
            this.txtbBashPath.Location = new System.Drawing.Point(8, 95);
            this.txtbBashPath.Margin = new System.Windows.Forms.Padding(2);
            this.txtbBashPath.Name = "txtbBashPath";
            this.txtbBashPath.Size = new System.Drawing.Size(336, 20);
            this.txtbBashPath.TabIndex = 11;
            this.txtbBashPath.TextChanged += new System.EventHandler(this.txtbBashPath_TextChanged);
            // 
            // lblBashPath
            // 
            this.lblBashPath.AutoSize = true;
            this.lblBashPath.Location = new System.Drawing.Point(6, 78);
            this.lblBashPath.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.lblBashPath.Name = "lblBashPath";
            this.lblBashPath.Size = new System.Drawing.Size(55, 13);
            this.lblBashPath.TabIndex = 10;
            this.lblBashPath.Text = "Bash path";
            // 
            // btnBStylePath
            // 
            this.btnBStylePath.Location = new System.Drawing.Point(347, 46);
            this.btnBStylePath.Margin = new System.Windows.Forms.Padding(2);
            this.btnBStylePath.Name = "btnBStylePath";
            this.btnBStylePath.Size = new System.Drawing.Size(35, 19);
            this.btnBStylePath.TabIndex = 9;
            this.btnBStylePath.Text = "...";
            this.btnBStylePath.UseVisualStyleBackColor = true;
            this.btnBStylePath.Click += new System.EventHandler(this.btnBStylePath_Click);
            // 
            // txtbBStylePath
            // 
            this.txtbBStylePath.Location = new System.Drawing.Point(8, 46);
            this.txtbBStylePath.Margin = new System.Windows.Forms.Padding(2);
            this.txtbBStylePath.Name = "txtbBStylePath";
            this.txtbBStylePath.Size = new System.Drawing.Size(336, 20);
            this.txtbBStylePath.TabIndex = 8;
            this.txtbBStylePath.TextChanged += new System.EventHandler(this.txtbBStylePath_TextChanged);
            // 
            // lblBStylePath
            // 
            this.lblBStylePath.AutoSize = true;
            this.lblBStylePath.Location = new System.Drawing.Point(6, 29);
            this.lblBStylePath.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.lblBStylePath.Name = "lblBStylePath";
            this.lblBStylePath.Size = new System.Drawing.Size(64, 13);
            this.lblBStylePath.TabIndex = 7;
            this.lblBStylePath.Text = "B-Style path";
            // 
            // checkBoxCppFormatOnSave
            // 
            this.checkBoxCppFormatOnSave.AutoSize = true;
            this.checkBoxCppFormatOnSave.Location = new System.Drawing.Point(8, 3);
            this.checkBoxCppFormatOnSave.Name = "checkBoxCppFormatOnSave";
            this.checkBoxCppFormatOnSave.Size = new System.Drawing.Size(99, 17);
            this.checkBoxCppFormatOnSave.TabIndex = 6;
            this.checkBoxCppFormatOnSave.Text = "Format on save";
            this.checkBoxCppFormatOnSave.UseVisualStyleBackColor = true;
            // 
            // tabControlOptions
            // 
            this.tabControlOptions.Controls.Add(this.tabPageCPP);
            this.tabControlOptions.Location = new System.Drawing.Point(3, 3);
            this.tabControlOptions.Name = "tabControlOptions";
            this.tabControlOptions.SelectedIndex = 0;
            this.tabControlOptions.Size = new System.Drawing.Size(394, 285);
            this.tabControlOptions.TabIndex = 3;
            // 
            // dlgBStyle
            // 
            this.dlgBStyle.AddExtension = false;
            this.dlgBStyle.FileName = "bstyle";
            this.dlgBStyle.Title = "Select B-Style";
            // 
            // dlgBash
            // 
            this.dlgBash.FileName = "bash.exe";
            this.dlgBash.Title = "Select bash executable";
            // 
            // errorProvider
            // 
            this.errorProvider.ContainerControl = this;
            // 
            // BStyleGeneralOptionsControl
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.tabControlOptions);
            this.Name = "BStyleGeneralOptionsControl";
            this.Size = new System.Drawing.Size(404, 302);
            this.tabPageCPP.ResumeLayout(false);
            this.tabPageCPP.PerformLayout();
            this.tabControlOptions.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.errorProvider)).EndInit();
            this.ResumeLayout(false);

        }


        #endregion

        private System.Windows.Forms.TabPage tabPageCPP;
        private System.Windows.Forms.CheckBox checkBoxCppFormatOnSave;
        private System.Windows.Forms.TabControl tabControlOptions;
        private System.Windows.Forms.Button btnBStylePath;
        private System.Windows.Forms.TextBox txtbBStylePath;
        private System.Windows.Forms.Label lblBStylePath;
        private System.Windows.Forms.Button btnBashPath;
        private System.Windows.Forms.TextBox txtbBashPath;
        private System.Windows.Forms.Label lblBashPath;
        private System.Windows.Forms.OpenFileDialog dlgBStyle;
        private System.Windows.Forms.OpenFileDialog dlgBash;
        private System.Windows.Forms.ErrorProvider errorProvider;
        private System.Windows.Forms.CheckBox checkBoxCppKeepTempFiles;
    }
}
