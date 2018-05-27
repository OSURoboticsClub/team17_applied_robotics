#NoEnv  ; Recommended for performance and compatibility with future AutoHotkey releases.
; #Warn  ; Enable warnings to assist with detecting common errors.
SendMode Input  ; Recommended for new scripts due to its superior speed and reliability.
SetWorkingDir %A_ScriptDir%  ; Ensures a consistent starting directory.

connect_status_box_x := 18
connect_status_box_y := 86
connect_status_green := "0x00FF00"

run_status_box_x := 13
run_status_box_y := 106
run_status_green := "0x00FF00"

error_status_box_x := 45
error_status_box_y := 431
error_status_grey := "0xFFFFFF"

connect_button_x := 73
connect_button_y := 88

run_button_x := 72
run_button_y := 106

clear_button_x := 88
clear_button_y := 446

should_exit = 0

Loop 
{
	If (should_exit)
	{
	ExitApp
	}
	
	IfWinNotExist, RobMaster
	{
		Run, C:\ORiN2\CAO\ProviderLib\DENSO\NetwoRC\Bin\RobMaster.exe
		Sleep, 3000
	}
	
	IfWinExist, RobMaster
	{
		WinActivate
	}
	
	IfWinActive, RobMaster
	{
		PixelGetColor, connect_status_color, %connect_status_box_x%, %connect_status_box_y%
		;MsgBox, Color is %connect_status_color%, other color is %connect_status_green% 
		
		If( connect_status_color <> connect_status_green )
		{
			Click, %connect_button_x%, %connect_button_y%
			Sleep, 250
		}
		
		PixelGetColor, error_status_color, %error_status_box_x%, %error_status_box_y%
		
		If( error_status_color <> error_status_grey )
		{
			Click, %clear_button_x%, %clear_button_y%
			Sleep, 250
		}
		else
		{
			PixelGetColor, run_status_color, %run_status_box_x%, %run_status_box_y%
			
			If( run_status_color <> run_status_green )
			{
				Click, %run_button_x%, %run_button_y%
				Sleep, 250
			
			}
		
		}
	}
	
	Sleep, 100
}

^q::
should_exit = 1
return

Return

