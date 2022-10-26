using System;
using Microsoft.VisualStudio;
using Microsoft.VisualStudio.Shell;
using Microsoft.VisualStudio.Shell.Interop;

namespace BStyleExtension
{
    public class DocumentEventListener : IDisposable, IVsRunningDocTableEvents3
    {
        private IVsRunningDocumentTable _table;
        private uint _cookie;
        private bool _isDisposed;

        public delegate int OnAfterSaveHandler(uint docCookie);
        public event OnAfterSaveHandler AfterSave;

        public DocumentEventListener(IVsRunningDocumentTable table)
        {
            _table = table;
            _table.AdviseRunningDocTableEvents(this, out _cookie);
        }

        public string GetDocumentName(uint docCookie)
        {
            string path;
            _table.GetDocumentInfo(docCookie, out _, out _, out _, out path, out _, out _, out _);
            return path;
        }

        public int OnAfterAttributeChange(uint docCookie, uint grfAttribs)
        {
            return VSConstants.S_OK;
        }

        public int OnAfterAttributeChangeEx(uint docCookie, uint grfAttribs, IVsHierarchy pHierOld, uint itemidOld, string pszMkDocumentOld, IVsHierarchy pHierNew, uint itemidNew, string pszMkDocumentNew)
        {
            return VSConstants.S_OK;
        }

        public int OnAfterDocumentWindowHide(uint docCookie, IVsWindowFrame pFrame)
        {
            return VSConstants.S_OK;
        }

        public int OnAfterFirstDocumentLock(uint docCookie, uint dwRDTLockType, uint dwReadLocksRemaining, uint dwEditLocksRemaining)
        {
            return VSConstants.S_OK;
        }

        public int OnAfterSave(uint docCookie)
        {
            if (AfterSave != null)
            {
                return AfterSave(docCookie);
            }
            return VSConstants.S_OK;
        }

        public int OnBeforeDocumentWindowShow(uint docCookie, int fFirstShow, IVsWindowFrame pFrame)
        {
            return VSConstants.S_OK;
        }

        public int OnBeforeLastDocumentUnlock(uint docCookie, uint dwRDTLockType, uint dwReadLocksRemaining, uint dwEditLocksRemaining)
        {
            return VSConstants.S_OK;
        }

        public int OnBeforeSave(uint docCookie)
        {
            return VSConstants.S_OK;
        }

        public void Dispose()
        {
            if (_isDisposed)
            {
                return;
            }

            _isDisposed = true;

            if (_table != null && _cookie != 0)
            {
                _table.UnadviseRunningDocTableEvents(_cookie);
                _cookie = 0;
            }
        }
    }
}