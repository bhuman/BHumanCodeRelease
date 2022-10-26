using System;
using System.Linq;
using System.Runtime.InteropServices;
using System.ComponentModel.Design;
using System.Text.RegularExpressions;
using System.Windows.Forms;
using Microsoft.VisualStudio;
using Microsoft.VisualStudio.Shell;
using Microsoft.VisualStudio.Shell.Interop;
using System.IO;
using System.Threading;
using Task = System.Threading.Tasks.Task;
using System.Collections.Generic;

namespace BStyleExtension
{
    [PackageRegistration(UseManagedResourcesOnly = true, AllowsBackgroundLoading = true)]
    [InstalledProductRegistration("#110", "#112", "3.0", IconResourceID = 400)]
    [ProvideMenuResource("Menus.ctmenu", 1)]
    [ProvideOptionPage(typeof(BStyleGeneralOptionsPage), "BStyle Formatter", "General", 1000, 1001, true)]
    [ProvideProfileAttribute(typeof(BStyleGeneralOptionsPage), "BStyle Formatter", "General", 1002, 1003, true)]
    [ProvideAutoLoad("{f1536ef8-92ec-443c-9ed7-fdadf150da82}", PackageAutoLoadFlags.BackgroundLoad)]
    [Guid(GuidList.GuidPkgString)]
    public sealed class BStyleExtensionPackage : AsyncPackage
    {
        private EnvDTE80.DTE2 _dte;
        private OleMenuCommand _formatSelMenuCommand;
        private OleMenuCommand _formatDocMenuCommand;
        private BStyleGeneralOptionsPage _dialog;
        private DocumentEventListener _documentEventListener;

        protected override async Task InitializeAsync(CancellationToken cancellationToken, IProgress<ServiceProgressData> progress)
        {
            if (await GetServiceAsync(typeof(IMenuCommandService)) is OleMenuCommandService mcs)
            {
                var id = new CommandID(GuidList.GuidCmdSet, (int)PkgCmdIDList.FormatDocumentCommand);
                _formatDocMenuCommand = new OleMenuCommand(FormatDocumentCallback, id);
                mcs.AddCommand(_formatDocMenuCommand);
                _formatDocMenuCommand.BeforeQueryStatus += OnBeforeQueryStatus;

                id = new CommandID(GuidList.GuidCmdSet, (int)PkgCmdIDList.FormatSelectionCommand);
                _formatSelMenuCommand = new OleMenuCommand(FormatSelectionCallback, id);
                mcs.AddCommand(_formatSelMenuCommand);
                _formatSelMenuCommand.BeforeQueryStatus += OnBeforeQueryStatus;
            }

            _dte = await GetServiceAsync(typeof(EnvDTE.DTE)) as EnvDTE80.DTE2;

            await JoinableTaskFactory.SwitchToMainThreadAsync();

            var table = await GetServiceAsync(typeof(SVsRunningDocumentTable)) as IVsRunningDocumentTable;
            _documentEventListener = new DocumentEventListener(table);
            _documentEventListener.AfterSave += OnAfterDocumentSave;

            _dialog = (BStyleGeneralOptionsPage)GetDialogPage(typeof(BStyleGeneralOptionsPage));
            if (String.IsNullOrEmpty(_dialog.CppBStylePath))
            {
                _dialog.CppBStylePath = SuggestBStylePath();
            }
            if (String.IsNullOrEmpty(_dialog.CppBashPath))
            {
                _dialog.CppBashPath = SuggestBashPath();
            }
        }

        private EnvDTE.TextDocument GetTextDocument(EnvDTE.Document doc)
        {
            if (doc == null || doc.ReadOnly)
            {
                return null;
            }

            var textDoc = doc.Object("TextDocument") as EnvDTE.TextDocument;

            return textDoc;
        }

        private int OnAfterDocumentSave(uint docCookie)
        {
            if (!_dialog.CppFormatOnSave)
            {
                return VSConstants.S_OK;
            }

            var doc = _dte.Documents.OfType<EnvDTE.Document>().FirstOrDefault(x => x.FullName == _documentEventListener.GetDocumentName(docCookie));

            FormatDocument(doc);

            return VSConstants.S_OK;
        }

        private void OnBeforeQueryStatus(object sender, EventArgs e)
        {
            var cmd = (OleMenuCommand)sender;

            cmd.Enabled = true;
        }

        private void FormatDocumentCallback(object sender, EventArgs e)
        {
            FormatDocument(_dte.ActiveDocument);
        }

        private void FormatDocument(EnvDTE.Document document)
        {
            EnvDTE.TextDocument textDoc = GetTextDocument(document);
            if (textDoc == null)
            {
                return;
            }

            EnvDTE.EditPoint sp = textDoc.StartPoint.CreateEditPoint();
            EnvDTE.EditPoint ep = textDoc.EndPoint.CreateEditPoint();
            string text = sp.GetText(ep);

            if (String.IsNullOrEmpty(text))
            {
                return;
            }

            string formattedText = FormatSourceFile(document.FullName, text);
            if (!String.IsNullOrEmpty(formattedText))
            {
                sp.ReplaceText(ep, formattedText, (int)EnvDTE.vsEPReplaceTextOptions.vsEPReplaceTextKeepMarkers);
            }
        }

        private void FormatSelectionCallback(object sender, EventArgs e)
        {
            var textDoc = GetTextDocument(_dte.ActiveDocument);

            FormatSelection(textDoc);
        }

        private void FormatSelection(EnvDTE.TextDocument textDoc)
        {
            if (textDoc == null)
            {
                return;
            }

            string newLineReplacement = "";

            if (textDoc.Selection.IsEmpty)
            {
                return;
            }

            EnvDTE.EditPoint sp = textDoc.Selection.TopPoint.CreateEditPoint();
            EnvDTE.EditPoint ep = textDoc.Selection.BottomPoint.CreateEditPoint();

            string text = textDoc.Selection.Text;

            int pos = 0;
            foreach (var c in text)
            {
                pos++;
                if (c != ' ' && c != '\t')
                {
                    break;
                }
            }

            if (pos > 0)
            {
                newLineReplacement = text.Substring(0, pos - 1);
            }

            string formattedText = Format(text);

            if (!String.IsNullOrEmpty(newLineReplacement))
            {
                string[] lines = Regex.Split(formattedText, "\r\n|\r|\n");

                for (int x = 0; x < lines.Length; x++)
                {
                    if (!string.IsNullOrEmpty(lines[x]))
                    {
                        lines[x] = newLineReplacement + lines[x];
                    }
                }

                formattedText = String.Join(Environment.NewLine, lines);
            }

            if (!String.IsNullOrEmpty(formattedText))
            {
                sp.ReplaceText(ep, formattedText, (int)EnvDTE.vsEPReplaceTextOptions.vsEPReplaceTextKeepMarkers);
            }
        }

        private string Format(string text)
        {
            var BStyle = new BStyleInterface();
            BStyle.ErrorRaised += OnBStyleErrorRaised;

            return BStyle.FormatSource(text, _dialog.CppBStylePath, _dialog.CppBashPath);
        }

        private string FormatSourceFile(string sourceFilePath, string text)
        {
            var BStyle = new BStyleInterface();
            BStyle.ErrorRaised += OnBStyleErrorRaised;

            return BStyle.FormatSourceFile(sourceFilePath, text, _dialog.CppBStylePath, _dialog.CppBashPath, _dialog.CppKeepTempFiles);
        }

        private void OnBStyleErrorRaised(object source, BStyleErrorArgs args)
        {
            MessageBox.Show(args.Message, "BStyle Formatter Error");
        }

        private String SuggestBStylePath()
        {
            if (String.IsNullOrEmpty(_dte.Solution.FullName))
                return null;

            String path = Path.GetFullPath(Path.Combine(Path.GetDirectoryName(_dte.Solution.FullName), @"..\..\..\Make\Common\bstyle"));
            return File.Exists(path) ? path : null;
        }

        private String SuggestBashPath()
        {
            String path = Environment.GetEnvironmentVariable("Path");
            foreach (string p in path.Split(';'))
            {
                String fullpath = Path.Combine(p, "bash.exe");
                if(File.Exists(fullpath))
                {
                    return fullpath;
                }
            }

            return null;
        }
    }
}
