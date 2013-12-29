// CallTool.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "resource.h"

HINSTANCE hInst;
HWND hDlg;
TCHAR cmd[MAX_PATH] = {0};
TCHAR* title = 0;
TCHAR* defaultCmd = 0;
TCHAR envHint[MAX_PATH] = {0};

BOOL dialogShow(HINSTANCE hInst);
INT_PTR WINAPI mainDialogProc(HWND hDlg, int msg, WPARAM wParam, LPARAM lParam);

void handleInitDialog();
void handleOKButton();
void handleCancelButton();
void handleCmdChange();
void updateOKButton();
BOOL saveCmds();
BOOL loadCmds();

int _tmain(int argc, _TCHAR* argv[])
{
  for(int i = 1; i < argc; ++i)
  {
    if(!_tcsicmp(argv[i], _T("/t")) && i + 1 < argc)
    {
      title = argv[++i];
      continue;
    }
    if(!_tcsicmp(argv[i], _T("/c")) && i + 1 < argc)
    {
      defaultCmd = argv[++i];
      continue;
    }

    TCHAR* eq = _tcsstr(argv[i], _T("="));
    if(eq && _tputenv(argv[i]) == 0)
    {/*
      TCHAR* next = _tcsinc(eq);
      *eq = '\0';
      _tcscat_s(envHint, _T("%"));
      _tcscat_s(envHint, argv[i]);
      _tcscat_s(envHint, _T("%="));
      _tcscat_s(envHint, next);
      _tcscat_s(envHint, _T(" "));
      */
    }
    else
    {
      _tcscat_s(envHint, argv[i]);
      _tcscat_s(envHint, _T(" "));
    }
  }

  {
    INITCOMMONCONTROLSEX icc;
    icc.dwSize = sizeof(INITCOMMONCONTROLSEX);
    icc.dwICC  = ICC_STANDARD_CLASSES;
    if(!InitCommonControlsEx(&icc))
      return -1;
  }

	dialogShow(GetModuleHandle(NULL));
  if(*cmd)
    return _tsystem(cmd);
  
  return 0;
}

BOOL dialogShow(HINSTANCE hInst)
{
  ::hInst = hInst;

  return (int)DialogBox(hInst, MAKEINTRESOURCE(IDD_MAIN_DIALOG), NULL, (DLGPROC)mainDialogProc);
}

INT_PTR WINAPI mainDialogProc(HWND hDlg, int msg, WPARAM wParam, LPARAM lParam)
{
  switch(msg)
  {
  case WM_INITDIALOG:
    ::hDlg = hDlg;
    handleInitDialog();
    return TRUE;

  case WM_COMMAND:
    {
      int wmId = LOWORD(wParam);
      int wmEvent = HIWORD(wParam);
      switch(wmId)
      {
      case IDOK:
        handleOKButton();
        return TRUE;
      case IDCANCEL:
        handleCancelButton();
			  return TRUE;
      case IDC_CMD_COMBO:
        switch(wmEvent)
        {
        case CBN_SELCHANGE:
          {
            HWND hCombo = (HWND)lParam;
            int sel = ComboBox_GetCurSel(hCombo);
            if(sel != CB_ERR)
            {
              TCHAR value[MAX_PATH];
              ComboBox_GetLBText(hCombo, sel, value);
              ComboBox_SetText(hCombo, value);
            }
          }
        case CBN_EDITCHANGE:
          handleCmdChange();
          return TRUE;
        }
        break;
      }
    }
    break;

  }
  return FALSE;
}

void handleInitDialog()
{
  // add icon to dialog
	HICON hIcon = LoadIcon(hInst, MAKEINTRESOURCE(IDI_MAIN_ICON));
  HICON hSmall = LoadIcon(hInst, MAKEINTRESOURCE(IDI_SMALL_ICON));
	SendMessage(hDlg, WM_SETICON, (WPARAM)ICON_BIG, (LPARAM)hIcon);
	SendMessage(hDlg, WM_SETICON, (WPARAM)ICON_SMALL, (LPARAM)hSmall);
	DestroyIcon(hIcon);
  DestroyIcon(hSmall);

  if(title)
    SetWindowText(hDlg, title);

  if(*envHint)
    SetDlgItemText(hDlg, IDC_ENV_STATIC, envHint);

  loadCmds();
  if(defaultCmd)
  {
    TCHAR value[MAX_PATH];
    GetDlgItemText(hDlg, IDC_CMD_COMBO, value, MAX_PATH);
    if(!*value)
      SetDlgItemText(hDlg, IDC_CMD_COMBO, defaultCmd);
  }

  updateOKButton();
}

void handleOKButton()
{
  HWND hCombo = GetDlgItem(hDlg, IDC_CMD_COMBO);
  GetWindowText(hCombo, cmd, MAX_PATH);

  int item = ComboBox_FindStringExact(hCombo, -1, cmd);
  if(item >= 0)
    ComboBox_DeleteString(hCombo, item);
  ComboBox_InsertString(hCombo, 0, cmd);
  while(ComboBox_GetCount(hCombo) > 10)
    ComboBox_DeleteString(hCombo, 10);

  saveCmds();

  EndDialog(hDlg, 0);
}

void handleCancelButton()
{
  EndDialog(hDlg, 0);
}

void handleCmdChange()
{
  updateOKButton();
}

void updateOKButton()
{
  TCHAR value[MAX_PATH];
  for(;;)
  {
    GetDlgItemText(hDlg, IDC_CMD_COMBO, value, MAX_PATH);
    if(*value == _T('\0'))
      break;
    
    EnableWindow(GetDlgItem(hDlg, IDOK), TRUE);
    return;
  }

  EnableWindow(GetDlgItem(hDlg, IDOK), FALSE);
}

BOOL saveCmds()
{
  HKEY hKey;
  if(RegCreateKey(HKEY_CURRENT_USER, _T("Software\\B-Human\\CallTool"), &hKey) != ERROR_SUCCESS)
    return FALSE;

  HWND hCombo = GetDlgItem(hDlg, IDC_CMD_COMBO);
  TCHAR value[MAX_PATH * 10];
  TCHAR* pos = value;
  for(int i = 0, count = ComboBox_GetCount(hCombo); i < count; ++i)
  {
    ComboBox_GetLBText(hCombo, i, pos);
    while(*pos)
      ++pos;
    ++pos;
  }

  RegSetValueEx(hKey, title ? title : _T(""), NULL, REG_MULTI_SZ, (LPBYTE)value, ((char*)pos) - ((char*)value));
  RegCloseKey(hKey);
  return TRUE;
}

BOOL loadCmds()
{
  HKEY hKey;
  if(RegOpenKey(HKEY_CURRENT_USER, _T("Software\\B-Human\\CallTool"), &hKey) != ERROR_SUCCESS)
    return FALSE;

  DWORD type;
  TCHAR value[MAX_PATH * 10];
  DWORD size = sizeof(value);
  if(RegQueryValueEx(hKey, title ? title : _T(""), NULL, &type, (LPBYTE)value, &size) == ERROR_SUCCESS)
    if(type == REG_MULTI_SZ)
    {
      HWND hCombo = GetDlgItem(hDlg, IDC_CMD_COMBO);
      TCHAR* end = (TCHAR*)(((char*)value) + size);
      value[end - value - 1] = '\0';
      for(TCHAR* pos = value; pos < end; )
      {
        if(pos == value)
          SetWindowText(hCombo, pos);
        ComboBox_AddString(hCombo, pos);
        while(*pos)
          ++pos;
        ++pos;
      }
    }

  RegCloseKey(hKey);
  return TRUE;
}
