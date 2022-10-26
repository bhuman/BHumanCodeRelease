using System;

namespace BStyleExtension
{
    public class BStyleErrorArgs : EventArgs
    {
        public string Message { get; set; }

        public BStyleErrorArgs(string message)
        {
            Message = message;
        }
    }
}
