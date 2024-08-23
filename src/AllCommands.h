/**********************************************************************************
 the C language function associated with commands, functions or operators should be
 declared here
**********************************************************************************/
#if !defined(INCLUDE_COMMAND_TABLE) && !defined(INCLUDE_TOKEN_TABLE)
// format:
//      void cmd_???(void)
//      void fun_???(void)
//      void op_???(void)

void cmd_clear(void);
void cmd_continue(void);
void cmd_delete(void);
void cmd_dim(void);
void cmd_do(void);
void cmd_else(void);
void cmd_end(void);
void cmd_endfun(void);
void cmd_endsub(void);
void cmd_erase(void);
void cmd_error(void);
void cmd_exit(void);
void cmd_exitfor(void);
void cmd_for(void);
void cmd_subfun(void);
void cmd_gosub(void);
void cmd_goto(void);
void cmd_if(void);
void cmd_inc(void);
void cmd_input(void);
void cmd_let(void);
void cmd_lineinput(void);
void cmd_list(void);
void cmd_load(void);
void cmd_loop(void);
void cmd_merge(void);
void cmd_chain(void);
void cmd_new(void);
void cmd_next(void);
void cmd_null(void);
void cmd_on(void);
void cmd_print(void);
void cmd_randomize(void);
void cmd_read(void);
void cmd_restore(void);
void cmd_return(void);
void cmd_run(void);
void cmd_save(void);
void cmd_troff(void);
void cmd_tron(void);
void cmd_trace(void);
void cmd_const(void);
void cmd_select(void);
void cmd_case(void);
void cmd_option(void);
void cmd_dump(void);
void cmd_call(void);
void cmd_execute(void);
void cmd_mid(void);
void cmd_null(void);
void cmd_open(void);
void cmd_close(void);
void cmd_files(void);
void cmd_mkdir(void);
void cmd_rmdir(void);
void cmd_chdir(void);
void cmd_kill(void);
void cmd_copy(void);
void cmd_name(void);
void cmd_exitmmb(void);
void cmd_pause(void);
void cmd_timer(void);
void cmd_copyright(void);
void cmd_seek(void);
void cmd_library(void);
void cmd_pio(void);
void cmd_date(void);
void cmd_time(void);
void cmd_flash(void);
void cmd_var(void);
void cmd_flush(void);
void cmd_disk(void);
void cmd_play(void);
void cmd_text(void);
void cmd_pixel(void);
void cmd_circle(void);
void cmd_line(void);
void cmd_box(void);
void cmd_rbox(void);
void cmd_arc(void) ;
void cmd_triangle(void);
void cmd_blit(void);
void cmd_cls(void);
void cmd_font(void);
void cmd_colour(void);
void cmd_refresh(void);
void cmd_polygon(void);
void cmd_gui(void);
void cmd_tile(void);
void cmd_mode(void);
void cmd_3D(void);
void cmd_framebuffer(void);
void cmd_edit(void);
void cmd_port(void);
void cmd_adc(void);
void cmd_ir(void);
void cmd_lcd(unsigned char *p);
void cmd_keypad(unsigned char *p);
void cmd_backlight(void);
void cmd_bitbang(void);
void cmd_sync(void);
void cmd_setpin(void);
void cmd_pulse(void);
void cmd_pwm(void);
void cmd_pin(void);
void cmd_i2c(void);
void cmd_i2c2(void);
void cmd_rtc(void);
void cmd_math(void);
void cmd_memory(void);
void cmd_autosave(void);
void cmd_option(void);
void cmd_pause(void);
void cmd_timer(void);
void cmd_date(void);
void cmd_time(void);
void cmd_ireturn(void);
void cmd_poke(void);
void cmd_settick(void);
void cmd_watchdog(void);
void cmd_cpu(void);
void cmd_cfunction(void);
void cmd_longString(void);
void cmd_sort(void);
void cmd_csubinterrupt(void);
void cmd_library(void);
void cmd_onewire(void);
void cmd_ds18b20(void);
void cmd_spi(void);
void cmd_spi2(void);
void cmd_xmodem(void);
void cmd_ctrlval(void);
void cmd_GUIpage(unsigned char *p);
void cmd_gamepad(unsigned char *tp);
void cmd_sprite(void);
void cmd_comment(void);
void cmd_endcomment(void);
void cmd_blitmemory(void);
void cmd_configure(void);
void cmd_colourmap(void);
#ifdef PICOMITEWEB
    void cmd_web(void);
#endif

void cmd_hcg(void);

void op_invalid(void);
void op_exp(void);
void op_mul(void);
void op_div(void);
void op_divint(void);
void op_add(void);
void op_subtract(void);
void op_mod(void);
void op_ne(void);
void op_gte(void);
void op_lte(void);
void op_lt(void);
void op_gt(void);
void op_equal(void);
void op_and(void);
void op_or(void);
void op_xor(void);
void op_not(void);
void op_shiftleft(void);
void op_shiftright(void);
void op_inv(void);
void fun_pin(void);
void fun_port(void);
void fun_distance(void);
void fun_pulsin(void);
void fun_rgb(void);
void fun_mmhres(void);
void fun_mmvres(void);
void fun_mmcharwidth(void);
void fun_mmcharheight(void);
void fun_at(void);
void fun_pixel(void);
void fun_getscanline(void);
void fun_3D(void);
void fun_sprite(void);
void fun_eof(void);
void fun_loc(void);
void fun_lof(void);
void fun_cwd(void);
void fun_inputstr(void);
void fun_mmfname(void);
void fun_dir(void);
void fun_date(void);
void fun_time(void);
void fun_timer(void);
void fun_device(void);
void fun_pio(void);
void fun_abs(void);
void fun_asc(void);
void fun_atn(void);
void fun_atan2(void);
void fun_bin(void);
void fun_chr(void);
void fun_cint(void);
void fun_cos(void);
void fun_deg(void);
void fun_exp(void);
void fun_fix(void);
void fun_hex(void);
void fun_inkey(void);
void fun_instr(void);
void fun_int(void);
void fun_lcase(void);
void fun_left(void);
void fun_len(void);
void fun_log(void);
void fun_errno(void);
void fun_errmsg(void);
void fun_mid(void);
void fun_oct(void);
void fun_peek(void);
void fun_pi(void);
void fun_pos(void);
void fun_rad(void);
void fun_right(void);
void fun_rnd(void);
void fun_sgn(void);
void fun_sin(void);
void fun_space(void);
void fun_sqr(void);
void fun_str(void);
void fun_string(void);
void fun_tab(void);
void fun_tan(void);
void fun_ucase(void);
void fun_val(void);
void fun_eval(void);
void fun_version(void);
void fun_asin(void);
void fun_acos(void);
void fun_field(void);
void fun_max(void);
void fun_min(void);
void fun_bin2str(void);
void fun_str2bin(void);
void fun_test(void);
void fun_bound(void);
void fun_ternary(void);
void fun_call(void);
void fun_GPS(void);
void fun_mmi2c(void);
void fun_math(void);
void fun_timer(void);
void fun_date(void);
void fun_time(void);
void fun_device(void);
void fun_keydown(void);
void fun_peek(void);
void fun_restart(void);
void fun_day(void);
void fun_info(void);
void fun_LLen(void);
void fun_LGetByte(void);
void fun_LGetStr(void);
void fun_LCompare(void);
void fun_LInstr(void);
void fun_epoch(void);
void fun_datetime(void);
void fun_json(void);
void cmd_update(void);
void fun_format(void);
void fun_mmOW(void);
void fun_ds18b20(void);
void fun_pixel(void);
void fun_getscanline(void);
void fun_spi(void);
void fun_spi2(void);
void fun_msgbox(void);
void fun_ctrlval(void);
void fun_mmhpos(void);
void fun_mmvpos(void);
#ifdef PICOMITEWEB
    void fun_json(void);
#endif
void fun_dev(void);
#endif

/**********************************************************************************
 All command tokens tokens (eg, PRINT, FOR, etc) should be inserted in this table
**********************************************************************************/
#ifdef INCLUDE_COMMAND_TABLE

	{ (unsigned char *)"Call",		T_CMD,				0, cmd_call	    },
	{ (unsigned char *)"HCG",		T_CMD,				0, cmd_hcg},
	{ (unsigned char *)"Clear",		T_CMD,				0, cmd_clear	},
	{ (unsigned char *)"Continue",  T_CMD,              0, cmd_continue	},
	{ (unsigned char *)"Data",		T_CMD,				0, cmd_null	    },
	{ (unsigned char *)"Dim",		T_CMD,				0, cmd_dim	    },
	{ (unsigned char *)"Do",		T_CMD,				0, cmd_do	},
	{ (unsigned char *)"Else If",		T_CMD,				0, cmd_else	},
	{ (unsigned char *)"End If",		T_CMD,				0, cmd_null	},
	{ (unsigned char *)"Exit Do",		T_CMD,				0, cmd_exit	},
	{ (unsigned char *)"ElseIf",		T_CMD,				0, cmd_else	},
	{ (unsigned char *)"Case Else",		T_CMD,				0, cmd_case	},
	{ (unsigned char *)"Else",		T_CMD,				0, cmd_else	},
	{ (unsigned char *)"Select Case",	T_CMD,				0, cmd_select	},
	{ (unsigned char *)"End Select",		T_CMD,				0, cmd_null	},
	{ (unsigned char *)"Case",		T_CMD,				0, cmd_case	},
	{ (unsigned char *)"EndIf",		T_CMD,				0, cmd_null	},
	{ (unsigned char *)"End Function",       T_CMD,                          0, cmd_endfun	},      // this entry must come before END and FUNCTION
	{ (unsigned char *)"End Sub",            T_CMD,                          0, cmd_return	},      // this entry must come before END and SUB
	{ (unsigned char *)"End",		T_CMD,				0, cmd_end	},
	{ (unsigned char *)"Erase",		T_CMD,				0, cmd_erase	},
	{ (unsigned char *)"Error",		T_CMD,				0, cmd_error	},
	{ (unsigned char *)"Exit For",           T_CMD,				0, cmd_exitfor	},      // this entry must come before EXIT and FOR
	{ (unsigned char *)"Exit Sub",           T_CMD,				0, cmd_return	},      // this entry must come before EXIT and SUB
	{ (unsigned char *)"Exit Function",      T_CMD,                          0, cmd_endfun	},      // this entry must come before EXIT and FUNCTION
	{ (unsigned char *)"Exit",		T_CMD,				0, cmd_exit	},
	{ (unsigned char *)"For",		T_CMD,				0, cmd_for	},
	{ (unsigned char *)"Function",           T_CMD,				0, cmd_subfun	},
	{ (unsigned char *)"GoSub",		T_CMD,				0, cmd_gosub	},
	{ (unsigned char *)"GoTo",		T_CMD,				0, cmd_goto	},
	{ (unsigned char *)"Inc",			T_CMD,				0, cmd_inc	},
	{ (unsigned char *)"If",			T_CMD,				0, cmd_if	},
	{ (unsigned char *)"Line Input",         T_CMD,				0, cmd_lineinput},      // this entry must come before INPUT
	{ (unsigned char *)"Input",		T_CMD,				0, cmd_input	},
	{ (unsigned char *)"Let",		T_CMD,				0, cmd_let	},
	{ (unsigned char *)"List",		T_CMD,				0, cmd_list	},
	{ (unsigned char *)"Load",		T_CMD,				0, cmd_load		},
	{ (unsigned char *)"Local",		T_CMD,				0, cmd_dim	},
	{ (unsigned char *)"Loop",		T_CMD,				0, cmd_loop	},
	{ (unsigned char *)"Next",		T_CMD,				0, cmd_next	},
	{ (unsigned char *)"On",			T_CMD,				0, cmd_on	},
	{ (unsigned char *)"Print",		T_CMD,				0, cmd_print	},
	{ (unsigned char *)"Randomize",          T_CMD,				0, cmd_randomize},
	{ (unsigned char *)"Read",		T_CMD,				0, cmd_read	},
	{ (unsigned char *)"Rem",		T_CMD,				0, cmd_null,	},
	{ (unsigned char *)"Restore",            T_CMD,				0, cmd_restore	},
	{ (unsigned char *)"Return",		T_CMD,				0, cmd_return,	},
	{ (unsigned char *)"Run",		T_CMD,				0, cmd_run	},
	{ (unsigned char *)"Save",		T_CMD,				0, cmd_save		},
    { (unsigned char *)"Static",		T_CMD,				0, cmd_dim		},
	{ (unsigned char *)"Sub",		T_CMD,				0, cmd_subfun   },
	{ (unsigned char *)"Trace",		T_CMD,				0, cmd_trace	},
	{ (unsigned char *)"While",		T_CMD,				0, cmd_do	},
	{ (unsigned char *)"Const",		T_CMD,				0, cmd_const	},
	{ (unsigned char *)"Execute",	T_CMD,				0, cmd_execute	},
	{ (unsigned char *)"MID$(",		T_CMD | T_FUN,		0, cmd_mid      },
	{ (unsigned char *)"/*",		T_CMD,				0, cmd_comment   },
	{ (unsigned char *)"*/",		T_CMD,				0, cmd_endcomment   },
	{ (unsigned char *)"Open",		T_CMD,				0, cmd_open		},
	{ (unsigned char *)"Close",		T_CMD,				0, cmd_close	},
	{ (unsigned char *)"Kill",		T_CMD,				0, cmd_kill		},
	{ (unsigned char *)"Rmdir",		T_CMD,				0, cmd_rmdir	},
	{ (unsigned char *)"Chdir",		T_CMD,				0, cmd_chdir	},
	{ (unsigned char *)"Mkdir",		T_CMD,				0, cmd_mkdir	},
	{ (unsigned char *)"Copy",		T_CMD,				0, cmd_copy		},
	{ (unsigned char *)"Rename",	T_CMD,				0, cmd_name		},
	{ (unsigned char *)"Seek",		T_CMD,				0, cmd_seek     },
	{ (unsigned char *)"Flash",		T_CMD,				0, cmd_flash    },
	{ (unsigned char *)"VAR",		T_CMD,				0, cmd_var     	},
	{ (unsigned char *)"Flush",		T_CMD,				0, cmd_flush    },
	{ (unsigned char *)"Drive",		T_CMD,				0, cmd_disk     },
	{ (unsigned char *)"Play",      T_CMD,				0, cmd_play	    },
    { (unsigned char *)"PIO",       T_CMD,              0, cmd_pio	    },
	{ (unsigned char *)"Text",           T_CMD,                      0, cmd_text	},
	{ (unsigned char *)"Pixel",          T_CMD,                      0, cmd_pixel	},
	{ (unsigned char *)"Circle",         T_CMD,                      0, cmd_circle	},
	{ (unsigned char *)"Line",           T_CMD,                      0, cmd_line	},
	{ (unsigned char *)"Box",            T_CMD,                      0, cmd_box	},
	{ (unsigned char *)"RBox",           T_CMD,                      0, cmd_rbox	},
	{ (unsigned char *)"CLS",            T_CMD,                      0, cmd_cls	},
	{ (unsigned char *)"Font",           T_CMD,                      0, cmd_font	},
	{ (unsigned char *)"Colour Map",         T_CMD,                      0, cmd_colourmap	},
  	{ (unsigned char *)"Triangle",       T_CMD,                      0, cmd_triangle   },
	{ (unsigned char *)"Arc",            T_CMD,                      0, cmd_arc	},
	{ (unsigned char *)"Polygon",        T_CMD,                  	 0, cmd_polygon	},
  	{ (unsigned char *)"FRAMEBUFFER",     T_CMD,                     0, cmd_framebuffer   },
	{ (unsigned char *)"Sprite",           T_CMD,                      0, cmd_sprite	},
	{ (unsigned char *)"Blit",           T_CMD,                      0, cmd_blit	},
    { (unsigned char *)"Edit",   T_CMD,              0, cmd_edit     },
    { (unsigned char *)"ADC",		T_CMD,			0, cmd_adc        },
    { (unsigned char *)"Pin(",		T_CMD | T_FUN,		0, cmd_pin          },
	{ (unsigned char *)"SetPin",		T_CMD,			0, cmd_setpin       },
	{ (unsigned char *)"Pulse",		T_CMD,			0, cmd_pulse        },
	{ (unsigned char *)"Port(",		T_CMD | T_FUN,		0, cmd_port	    },
	{ (unsigned char *)"IR",                 T_CMD,			0, cmd_ir           },
	{ (unsigned char *)"Blit Memory",           T_CMD,                      0, cmd_blitmemory	},
#ifdef PICOMITE
  	{ (unsigned char *)"GUI",            T_CMD,                      0, cmd_gui   },
#else
  	{ (unsigned char *)"GUI",            T_CMD,                      0, cmd_guiMX170   },
#endif
	{ (unsigned char *)"Device",              T_CMD,			0, cmd_bitbang        },
	{ (unsigned char *)"PWM",		T_CMD,		0, cmd_pwm		},
	{ (unsigned char *)"CSub",           T_CMD,              0, cmd_cfunction},
	{ (unsigned char *)"End CSub",       T_CMD,              0, cmd_null     },
    { (unsigned char *)"SYNC",              T_CMD,			0, cmd_sync        },
	{ (unsigned char *)"I2C",	T_CMD,		0, cmd_i2c              },
	{ (unsigned char *)"I2C2",	T_CMD,		0, cmd_i2c2              },
  	{ (unsigned char *)"RTC",    T_CMD,      0, cmd_rtc              },
	{ (unsigned char *)"Math",		T_CMD,				0, cmd_math		},
	{ (unsigned char *)"Memory",		T_CMD,				0, cmd_memory	},
	{ (unsigned char *)"Option",			T_CMD,				0, cmd_option	},
	{ (unsigned char *)"Pause",			T_CMD,				0, cmd_pause	},
	{ (unsigned char *)"Timer",			T_CMD | T_FUN,      0, cmd_timer	},
	{ (unsigned char *)"Date$",			T_CMD | T_FUN,      0, cmd_date		},
	{ (unsigned char *)"Time$",			T_CMD | T_FUN,      0, cmd_time		},
	{ (unsigned char *)"IReturn",		T_CMD,				0, cmd_ireturn 	},
	{ (unsigned char *)"Poke",			T_CMD,				0, cmd_poke		},
	{ (unsigned char *)"SetTick",		T_CMD,				0, cmd_settick 	},
	{ (unsigned char *)"WatchDog",		T_CMD,				0, cmd_watchdog	},
	{ (unsigned char *)"CPU",			T_CMD,				0, cmd_cpu 	},
	{ (unsigned char *)"Sort",			T_CMD,				0, cmd_sort 	},
    { (unsigned char *)"DefineFont",     T_CMD,				0, cmd_cfunction},
    { (unsigned char *)"End DefineFont", T_CMD,				0, cmd_null 	},
	{ (unsigned char *)"LongString",	T_CMD,				0, cmd_longString	},
	{ (unsigned char *)"Interrupt", 	T_CMD,              	0, cmd_csubinterrupt},
	{ (unsigned char *)"Library",       T_CMD,              0, cmd_library     },
	{ (unsigned char *)"OneWire",	T_CMD,		0, cmd_onewire      },
	{ (unsigned char *)"TEMPR START", T_CMD,	0, cmd_ds18b20      },
	{ (unsigned char *)"SPI",	T_CMD,				0, cmd_spi	},
	{ (unsigned char *)"SPI2",	T_CMD,					0, cmd_spi2	},
    { (unsigned char *)"XModem",     T_CMD,              0, cmd_xmodem   },
	{ (unsigned char *)"Cat",			T_CMD,				0, cmd_inc	},
	{ (unsigned char *)"Color",         T_CMD,                      0, cmd_colour	},
	{ (unsigned char *)"Files",		T_CMD,				0, cmd_files	},
	{ (unsigned char *)"New",		T_CMD,				0, cmd_new	},
	{ (unsigned char *)"Autosave",		T_CMD,				0, cmd_autosave	},
#ifdef PICOMITEVGA
  	{ (unsigned char *)"TILE",            T_CMD,                     0, cmd_tile   },
  	{ (unsigned char *)"MODE",            T_CMD,                     0, cmd_mode   },
#else
    { (unsigned char *)"Refresh",         T_CMD,                      0, cmd_refresh },
#endif
#ifdef PICOMITE
	{ (unsigned char *)"Backlight",		T_CMD,		0, cmd_backlight		},
	{ (unsigned char *)"CtrlVal(",       T_CMD | T_FUN,              0, cmd_ctrlval    },
#endif
#ifdef PICOMITEWEB
    { (unsigned char *)"NOP", T_CMD,				0, cmd_null 	}, //padding to keep tokens the same
	{ (unsigned char *)"Backlight",		T_CMD,		0, cmd_backlight		},
    { (unsigned char *)"WEB",       T_CMD,              0, cmd_web	    },
#else
    { (unsigned char *)"Draw3D",         T_CMD,                      0, cmd_3D },
#endif
#ifndef USBKEYBOARD
	{ (unsigned char *)"Update Firmware",		T_CMD,				0, cmd_update},
#endif
	{ (unsigned char *)"Configure",		T_CMD,				0, cmd_configure	},
	{ (unsigned char *)"Colour",         T_CMD,                      0, cmd_colour	},
    { (unsigned char *)"",   0,                  0, cmd_null,    }                   // this dummy entry is always at the end
#endif
/**********************************************************************************
 All other tokens (keywords, functions, operators) should be inserted in this table
**********************************************************************************/
#ifdef INCLUDE_TOKEN_TABLE
	{ (unsigned char *)"^",			T_OPER | T_NBR | T_INT,		0, op_exp		},
	{ (unsigned char *)"*",			T_OPER | T_NBR | T_INT,		1, op_mul		},
	{ (unsigned char *)"/",			T_OPER | T_NBR,                 1, op_div		},
	{ (unsigned char *)"\\",			T_OPER | T_INT,			1, op_divint            },
	{ (unsigned char *)"Mod",		T_OPER | T_INT,			1, op_mod		},
	{ (unsigned char *)"+",			T_OPER | T_NBR | T_INT | T_STR, 2, op_add		},
	{ (unsigned char *)"-",			T_OPER | T_NBR | T_INT,		2, op_subtract          },
	{ (unsigned char *)"Not",		T_OPER | T_NBR | T_INT,			3, op_not		},
	{ (unsigned char *)"INV",			T_OPER | T_NBR | T_INT,			3, op_inv		},
	{ (unsigned char *)"<<",			T_OPER | T_INT,                 4, op_shiftleft		},
	{ (unsigned char *)">>",			T_OPER | T_INT,                 4, op_shiftright	},
	{ (unsigned char *)"<>",			T_OPER | T_NBR | T_INT | T_STR, 5, op_ne		},
	{ (unsigned char *)">=",			T_OPER | T_NBR | T_INT | T_STR, 5, op_gte		},
	{ (unsigned char *)"<=",			T_OPER | T_NBR | T_INT | T_STR, 5, op_lte		},
	{ (unsigned char *)"<",			T_OPER | T_NBR | T_INT | T_STR, 5, op_lt		},
	{ (unsigned char *)">",			T_OPER | T_NBR | T_INT | T_STR, 5, op_gt		},
	{ (unsigned char *)"=",			T_OPER | T_NBR | T_INT | T_STR, 6, op_equal		},
	{ (unsigned char *)"And",		T_OPER | T_INT,			7, op_and		},
	{ (unsigned char *)"Or",			T_OPER | T_INT,			7, op_or		},
	{ (unsigned char *)"Xor",		T_OPER | T_INT,			7, op_xor		},
	{ (unsigned char *)"ACos(",		T_FUN  | T_NBR, 	        0, fun_acos		},
	{ (unsigned char *)"Abs(",		T_FUN  | T_NBR | T_INT, 	0, fun_abs		},
	{ (unsigned char *)"Asc(",		T_FUN  | T_INT,			0, fun_asc		},
	{ (unsigned char *)"ASin(",		T_FUN  | T_NBR,			0, fun_asin		},
	{ (unsigned char *)"Atn(",		T_FUN  | T_NBR,			0, fun_atn		},
	{ (unsigned char *)"Atan2(",		T_FUN  | T_NBR,			0, fun_atan2	},
	{ (unsigned char *)"Bin$(",		T_FUN  | T_STR,			0, fun_bin		},
	{ (unsigned char *)"Bound(",		T_FUN  | T_INT,			0, fun_bound	},
	{ (unsigned char *)"Choice(",	T_FUN | T_STR | T_INT | T_NBR,		0, fun_ternary	},
	{ (unsigned char *)"Chr$(",		T_FUN  | T_STR,			0, fun_chr,		},
	{ (unsigned char *)"Cint(",		T_FUN  | T_INT,			0, fun_cint		},
	{ (unsigned char *)"Cos(",		T_FUN  | T_NBR,			0, fun_cos		},
	{ (unsigned char *)"Deg(",		T_FUN  | T_NBR,			0, fun_deg		},
	{ (unsigned char *)"Exp(",		T_FUN  | T_NBR,			0, fun_exp		},
	{ (unsigned char *)"Field$(",    T_FUN  | T_STR,			0, fun_field    },
	{ (unsigned char *)"Fix(",		T_FUN  | T_INT,			0, fun_fix		},
	{ (unsigned char *)"Hex$(",		T_FUN  | T_STR,			0, fun_hex		},
	{ (unsigned char *)"Inkey$",	T_FNA  | T_STR,         0, fun_inkey    },
	{ (unsigned char *)"Instr(",		T_FUN  | T_INT,			0, fun_instr},
	{ (unsigned char *)"Int(",		T_FUN  | T_INT,			0, fun_int		},
	{ (unsigned char *)"LCase$(",            T_FUN  | T_STR,			0, fun_lcase            },
	{ (unsigned char *)"Left$(",		T_FUN  | T_STR,			0, fun_left		},
	{ (unsigned char *)"Len(",		T_FUN  | T_INT,			0, fun_len		},
	{ (unsigned char *)"Log(",		T_FUN  | T_NBR,			0, fun_log		},
	{ (unsigned char *)"Mid$(",		T_FUN  | T_STR,			0, fun_mid		},
	{ (unsigned char *)"MM.Errno",	T_FNA | T_INT,		0, fun_errno	},
  	{ (unsigned char *)"MM.ErrMsg$", T_FNA  | T_STR,         0, fun_errmsg   },
	{ (unsigned char *)"MM.Ver",		T_FNA  | T_NBR,			0, fun_version          },
	{ (unsigned char *)"Oct$(",		T_FUN  | T_STR,			0, fun_oct		},
	{ (unsigned char *)"Pi",			T_FNA  | T_NBR,			0, fun_pi		},
	{ (unsigned char *)"Pos",		T_FNA  | T_INT,                 0, fun_pos		},
	{ (unsigned char *)"Rad(",		T_FUN  | T_NBR,			0, fun_rad		},
	{ (unsigned char *)"Right$(",            T_FUN  | T_STR,			0, fun_right            },
	{ (unsigned char *)"Rnd(",		T_FUN  | T_NBR,			0, fun_rnd		},        // this must come before Rnd - without bracket
	{ (unsigned char *)"Rnd",		T_FNA  | T_NBR,			0, fun_rnd		},        // this must come after Rnd(
	{ (unsigned char *)"Sgn(",		T_FUN  | T_INT,			0, fun_sgn		},
	{ (unsigned char *)"Sin(",		T_FUN  | T_NBR,			0, fun_sin		},
	{ (unsigned char *)"Space$(",            T_FUN  | T_STR,			0, fun_space            },
	{ (unsigned char *)"Sqr(",		T_FUN  | T_NBR,			0, fun_sqr		},
	{ (unsigned char *)"Str$(",		T_FUN  | T_STR,			0, fun_str		},
	{ (unsigned char *)"String$(",           T_FUN  | T_STR,			0, fun_string           },
	{ (unsigned char *)"Tab(",		T_FUN  | T_STR,                 0, fun_tab,		},
	{ (unsigned char *)"Tan(",		T_FUN  | T_NBR,			0, fun_tan		},
	{ (unsigned char *)"UCase$(",            T_FUN  | T_STR,			0, fun_ucase            },
	{ (unsigned char *)"Val(",		T_FUN  | T_NBR | T_INT,		0, fun_val		},
	{ (unsigned char *)"Eval(",		T_FUN  | T_NBR | T_INT | T_STR,	0, fun_eval		},
	{ (unsigned char *)"Max(",		T_FUN  | T_NBR,	 		0, fun_max		},
	{ (unsigned char *)"Min(",		T_FUN  | T_NBR,			0, fun_min		},
	{ (unsigned char *)"Bin2str$(",  T_FUN  | T_STR,			0, fun_bin2str  },
	{ (unsigned char *)"Str2bin(",	T_FUN  | T_NBR | T_INT,	0, fun_str2bin	},
	{ (unsigned char *)"Call(",		T_FUN | T_STR | T_INT | T_NBR,		0, fun_call	},
	{ (unsigned char *)"For",		T_NA,				0, op_invalid	},
	{ (unsigned char *)"Else",		T_NA,				0, op_invalid	},
	{ (unsigned char *)"GoSub",		T_NA,				0, op_invalid	},
	{ (unsigned char *)"GoTo",		T_NA,				0, op_invalid	},
	{ (unsigned char *)"Step",		T_NA,				0, op_invalid	},
	{ (unsigned char *)"Then",		T_NA,				0, op_invalid	},
	{ (unsigned char *)"To",		T_NA,				0, op_invalid	},
	{ (unsigned char *)"Until",		T_NA,				0, op_invalid	},
	{ (unsigned char *)"While",		T_NA,				0, op_invalid	},
  	{ (unsigned char *)"Eof(",   T_FUN | T_INT,      0, fun_eof      },
  	{ (unsigned char *)"Loc(",   T_FUN | T_INT,      0, fun_loc      },
  	{ (unsigned char *)"Lof(",   T_FUN | T_INT,      0, fun_lof      },
	{ (unsigned char *)"Cwd$",		T_FNA | T_STR,		0, fun_cwd		},
	{ (unsigned char *)"As",			T_NA,			0, op_invalid	},
	{ (unsigned char *)"Input$(",	T_FUN | T_STR,		0, fun_inputstr	},
	{ (unsigned char *)"Dir$(",		T_FUN | T_STR,		0, fun_dir		},
	{ (unsigned char *)"Pio(",		T_FUN  | T_INT,			0, fun_pio		},
	{ (unsigned char *)"RGB(",           	T_FUN | T_INT,		0, fun_rgb	        },
	{ (unsigned char *)"Pixel(",           	T_FUN | T_INT,		0, fun_pixel	        },
	{ (unsigned char *)"MM.HRes",	    	T_FNA | T_INT,		0, fun_mmhres 	    },
	{ (unsigned char *)"MM.VRes",	    	T_FNA | T_INT,		0, fun_mmvres 	    },
	{ (unsigned char *)"@(",				T_FUN | T_STR,		0, fun_at		},
	{ (unsigned char *)"Pin(",		T_FUN | T_NBR | T_INT,	0, fun_pin		},
	{ (unsigned char *)"Port(",		T_FUN | T_INT,		0, fun_port		},
	{ (unsigned char *)"Distance(",		T_FUN | T_NBR,		0, fun_distance		},
	{ (unsigned char *)"Pulsin(",		T_FUN | T_INT,		0, fun_pulsin		},
	{ (unsigned char *)"GPS(",	    T_FUN | T_NBR | T_INT| T_STR,		0, fun_GPS	},
	{ (unsigned char *)"MM.I2C",	T_FNA | T_INT,	0, fun_mmi2c		},
	{ (unsigned char *)"Math(",	    T_FUN | T_NBR | T_INT,		0, fun_math	},
	{ (unsigned char *)"Timer",	T_FNA | T_NBR ,		0, fun_timer	},
	{ (unsigned char *)"LInStr(",		T_FUN | T_INT,		0, fun_LInstr		},
	{ (unsigned char *)"LCompare(",		T_FUN | T_INT,		0, fun_LCompare		},
	{ (unsigned char *)"LLen(",		T_FUN | T_INT,		0, fun_LLen		},
	{ (unsigned char *)"LGetStr$(",		T_FUN | T_STR,		0, fun_LGetStr		},
	{ (unsigned char *)"LGetByte(",		T_FUN | T_INT,		0, fun_LGetByte		},
	{ (unsigned char *)"Date$",	T_FNA | T_STR,		0, fun_date	},
	{ (unsigned char *)"Day$(",	T_FUN | T_STR,		0, fun_day	},
	{ (unsigned char *)"Peek(",		T_FUN  | T_INT | T_STR | T_NBR,			0, fun_peek		},
	{ (unsigned char *)"Time$",	T_FNA | T_STR,		0, fun_time	},
	{ (unsigned char *)"MM.Device$",	T_FNA | T_STR,		0, fun_device   },
	{ (unsigned char *)"MM.Watchdog",T_FNA | T_INT,		0, fun_restart	},
	{ (unsigned char *)"Epoch(",		T_FUN  | T_INT,			0, fun_epoch		},
	{ (unsigned char *)"DateTime$(",		T_FUN | T_STR,		0, fun_datetime		},
	{ (unsigned char *)"MM.Info(",		T_FUN | T_INT  | T_NBR| T_STR,		0, fun_info		},
	{ (unsigned char *)"Format$(",	T_FUN  | T_STR,			0, fun_format	},
	{ (unsigned char *)"MM.OneWire",	T_FNA | T_INT,	0, fun_mmOW         },
	{ (unsigned char *)"TEMPR(",	T_FUN | T_NBR,	0, fun_ds18b20      },
	{ (unsigned char *)"SPI(",	T_FUN | T_INT,		0, fun_spi,	},
	{ (unsigned char *)"SPI2(",	T_FUN | T_INT,		0, fun_spi2,	},
	{ (unsigned char *)"DEVICE(",	T_FUN | T_INT| T_NBR | T_STR,		0, fun_dev,	},
	{ (unsigned char*)"sprite(",	    T_FUN | T_INT | T_NBR,		0, fun_sprite },
#ifdef USBKEYBOARD
	{ (unsigned char*)"KeyDown(",    T_FUN | T_INT,		0, fun_keydown	},
#endif	
#ifdef PICOMITEVGA
	{ (unsigned char*)"DRAW3D(",	    T_FUN | T_INT,		0, fun_3D, },
	{ (unsigned char *)"GetScanLine",	    	T_FNA | T_INT,		0, fun_getscanline 	    },
#else
  	{ (unsigned char *)"Touch(",       T_FUN | T_INT,        0, fun_touch  },
#endif
#ifdef PICOMITEWEB
	{ (unsigned char *)"Json$(",		T_FUN | T_STR,          0, fun_json		},
#endif
#ifdef PICOMITE
	  { (unsigned char *)"MsgBox(",        T_FUN | T_INT,              0, fun_msgbox     },
	  { (unsigned char *)"CtrlVal(",       T_FUN | T_NBR | T_STR,      0, fun_ctrlval    },
#endif
    { (unsigned char *)"",   0,                  0, cmd_null,    }                   // this dummy entry is always at the end

#endif

