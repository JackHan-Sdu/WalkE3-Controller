#!/usr/bin/env python3
"""
配置文件编辑器 - GUI工具
用于可视化和编辑config.yaml中的所有配置项，一键生成配置文件

作者: Han Jiang (jh18954242606@163.com)
日期: 2025-12
"""

import os
import sys
import yaml
import tkinter as tk
import tkinter.font as tkfont
from tkinter import ttk, filedialog, messagebox, scrolledtext
from typing import Dict, Any, List, Optional, Tuple
from datetime import datetime
import random

# 尝试导入PIL用于图片处理
try:
    from PIL import Image, ImageTk, ImageEnhance
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False


class ConfigEditor:
    def __init__(self, root, config_file: Optional[str] = None):
        self.root = root
        self.root.title("人形机器人控制框架 - 配置文件编辑器")
        self.root.geometry("1300x850")
        
        self.config_file = config_file or "config.yaml"
        self.config_data: Dict[str, Any] = {}
        self.widgets: Dict[str, Any] = {}
        
        # 自定义主题和字体设置（初始化为None，使用默认值）
        self.custom_theme: Optional[str] = None
        self.custom_font_family: Optional[str] = None
        
        # 鼓励话语列表
        self.encouraging_messages = [
            # 通用鼓励话语
            ("💪", "相信自己，你能做到！"),
            ("🚀", "每一次尝试都是进步的开始"),
            ("⭐", "坚持就是胜利，加油！"),
            ("✨", "你的努力正在闪闪发光"),
            ("🎯", "专注目标，持续前进"),
            ("💡", "好的想法值得被实现"),
            ("🌈", "风雨过后总会有彩虹"),
            ("🌱", "每一个挑战都是成长的机会"),
            ("🎨", "用代码创造美好的未来"),
            ("⚡", "保持专注，效率倍增"),
            ("🎉", "完成比完美更重要"),
            ("🏆", "成功属于坚持不懈的人"),
            ("🌻", "向阳而生，充满希望"),
            
            # 工程师专属鼓励（Bug相关）
            ("🔧", "每个bug都是成长的机会，调试让我们更强大"),
            ("🐛", "找到bug的瞬间，就像发现了隐藏的宝藏"),
            ("💻", "优秀的工程师不是不写bug，而是快速修复bug"),
            ("🔍", "细心调试，耐心排查，代码会越来越优雅"),
            ("⚙️", "bug是代码的语言，听懂它就能写出更好的程序"),
            ("🛠️", "从bug中学到的，比从成功中学到的更多"),
            ("📝", "每一次debug都是一次深刻的学习"),
            ("🎓", "调试是一门艺术，你正在成为大师"),
            ("💡", "最复杂的bug往往有最简单的解决方案"),
            ("🔬", "像科学家一样思考，像工程师一样行动"),
            ("⚡", "快速定位bug的能力，是优秀工程师的标配"),
            ("🎯", "精准定位问题，优雅解决问题"),
            
            # 优宝特机器人公司祝福
            ("🔥", "加油优宝特，一定能上市！"),
            ("🤖", "优宝特人型机器人，引领未来智能时代"),
            ("🏔️", "人型机器人行者泰山会，登峰造极，勇攀高峰"),
            ("🚶", "行者泰山，稳健前行，走向世界舞台"),
            ("🌟", "优宝特机器人，让中国智造闪耀全球"),
            ("🎖️", "优宝特团队，技术精湛，追求卓越"),
            ("🌏", "优宝特机器人，服务全球，创造价值"),
            ("💎", "优宝特品质，精益求精，匠心独运"),
            ("🎊", "优宝特机器人，为人类进步贡献力量"),
            ("🚀", "优宝特加速前行，迈向更广阔的未来"),
            ("🏅", "优宝特技术领先，产品卓越，团队优秀"),
            ("🌠", "优宝特机器人，让科技温暖世界"),
            ("🎁", "优宝特产品，改变生活，改变未来"),
            ("⭐", "优宝特之星，在机器人领域熠熠生辉"),
            ("🎈", "优宝特梦想，从每一行代码开始实现"),
            ("🌈", "优宝特之路，虽有挑战，但前途光明"),
            ("🌲", "行者泰山，根深叶茂，基业长青"),
            ("🏆", "优宝特团队，团结协作，共创辉煌"),
            ("🎯", "优宝特目标清晰，步伐坚定，未来可期"),
            ("💫", "优宝特机器人，用技术创新驱动行业发展"),
        ]
        
        # 设置窗口图标和样式（必须在create_ui之前调用）
        self.setup_styles()
        
        # 定义配置顺序：非状态机配置在前，状态机配置在后
        self.non_fsm_sections = [
            "motor_communication",
            "simulation",
            "robot_control_parameters",
            "logging",
            "gamepad",
            "algorithm_launcher",
            "imu",
            "safety_checker",
        ]
        
        self.fsm_sections = [
            "rl_walk",
            "rl_dance",
            "rl_hybrid",
            "recovery_stand",
            "development",
        ]
        
        self.all_sections = self.non_fsm_sections + self.fsm_sections
        
        # 先加载配置文件（如果存在），这样UI创建时就有数据了
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    self.config_data = yaml.safe_load(f) or {}
            except Exception as e:
                print(f"加载配置文件失败: {e}")
                self.config_data = {}
        
        # 确保所有部分都存在
        for section in self.all_sections:
            if section not in self.config_data:
                self.config_data[section] = {}
        
        # 创建界面
        self.create_ui()
    
    def setup_styles(self):
        """设置界面样式和主题"""
        style = ttk.Style()
        
        # 尝试使用现代主题（如果已有自定义主题设置，使用它）
        if hasattr(self, 'custom_theme') and self.custom_theme:
            try:
                style.theme_use(self.custom_theme)
            except:
                try:
                    style.theme_use('clam')
                except:
                    pass
        else:
            try:
                style.theme_use('clam')  # 使用clam主题，更现代
            except:
                try:
                    style.theme_use('alt')
                except:
                    pass  # 使用默认主题
        
        # 配置字体 - 使用跨平台字体，自动检测可用字体
        # 注意：必须在root窗口创建后才能获取字体列表
        # 所有字体都不加粗，保持清晰
        # 如果已有自定义字体设置，使用它；否则使用默认检测
        if hasattr(self, 'custom_font_family') and self.custom_font_family:
            self.font_title = (self.custom_font_family, 13, 'normal')
            self.font_large = (self.custom_font_family, 11, 'normal')
            self.font_normal = (self.custom_font_family, 9, 'normal')
            self.font_small = (self.custom_font_family, 8, 'normal')
        else:
            self.font_large = self.get_available_font(11, 'normal')
            self.font_normal = self.get_available_font(9, 'normal')
            self.font_small = self.get_available_font(8, 'normal')
            self.font_title = self.get_available_font(13, 'normal')
        
        # 打印使用的字体信息（用于调试）
        print(f"[字体配置] 标题: {self.font_title[0]} {self.font_title[1]}pt, 正常: {self.font_normal[0]} {self.font_normal[1]}pt, 小: {self.font_small[0]} {self.font_small[1]}pt")
        
        # 颜色主题
        self.colors = {
            'bg_main': '#F5F5F5',
            'bg_section': '#FFFFFF',
            'bg_button': '#4A90E2',
            'bg_button_hover': '#357ABD',
            'fg_text': '#2C3E50',
            'fg_label': '#34495E',
            'border': '#BDC3C7',
            'accent': '#3498DB',
            'success': '#27AE60',
            'warning': '#F39C12',
        }
        
        # 创建Font对象用于ttk组件（使用实际字体名称）
        try:
            # 正确解包字体元组: (family, size, weight)
            self.font_title_obj = tkfont.Font(family=self.font_title[0], size=self.font_title[1], weight=self.font_title[2])
            self.font_large_obj = tkfont.Font(family=self.font_large[0], size=self.font_large[1], weight=self.font_large[2])
            self.font_normal_obj = tkfont.Font(family=self.font_normal[0], size=self.font_normal[1], weight=self.font_normal[2])
            self.font_small_obj = tkfont.Font(family=self.font_small[0], size=self.font_small[1], weight=self.font_small[2])
        except Exception as e:
            print(f"[警告] 字体创建失败: {e}")
            print(f"  尝试使用字体: 标题={self.font_title}, 正常={self.font_normal}")
            # 使用系统默认字体作为备选
            try:
                self.font_title_obj = tkfont.Font(size=13, weight='normal')
                self.font_large_obj = tkfont.Font(size=11, weight='normal')
                self.font_normal_obj = tkfont.Font(size=9)
                self.font_small_obj = tkfont.Font(size=8)
            except:
                # 最后的备选方案
                self.font_title_obj = tkfont.nametofont('TkDefaultFont')
                self.font_large_obj = tkfont.nametofont('TkDefaultFont')
                self.font_normal_obj = tkfont.nametofont('TkDefaultFont')
                self.font_small_obj = tkfont.nametofont('TkDefaultFont')
        
        # 配置样式 - ttk组件需要使用Font对象
        style.configure('Title.TLabel', font=self.font_title_obj, foreground=self.colors['fg_text'])
        style.configure('Section.TLabel', font=self.font_large_obj, foreground=self.colors['fg_label'])
        style.configure('Normal.TLabel', font=self.font_normal_obj, foreground=self.colors['fg_text'])
        style.configure('Primary.TButton', font=self.font_normal_obj, padding=(10, 5))
        style.configure('Action.TButton', font=self.font_normal_obj, padding=(8, 4))
        
        # 配置Notebook样式 - 标签页字体必须使用Font对象
        style.configure('TNotebook', background=self.colors['bg_main'])
        style.configure('TNotebook.Tab', font=self.font_normal_obj, padding=(15, 8))
        
        # Menu字体 - 使用Font对象而不是字符串（因为字体名可能包含空格）
        self.menu_font_obj = self.font_normal_obj
        
        # 配置Entry样式
        style.configure('TEntry', fieldbackground='white', borderwidth=1, padding=3)
        
        # 配置Frame样式
        style.configure('Card.TFrame', background=self.colors['bg_section'], relief='flat')
        style.configure('Section.TLabelframe', font=self.font_large_obj, foreground=self.colors['fg_label'])
    
    def get_available_font(self, size, weight='normal') -> Tuple[str, int, str]:
        """获取可用的字体，按优先级尝试多个字体"""
        # 获取系统可用字体（使用已有的root窗口）
        try:
            available_fonts_list = list(tkfont.families())
            available_fonts_lower = [f.lower() for f in available_fonts_list]
        except:
            available_fonts_list = []
            available_fonts_lower = []
        
        # 字体列表（按优先级排序，跨平台）
        # 首先尝试系统中实际存在的中文字体
        font_candidates = []
        
        # 检查系统中是否有中文字体（song ti, fangsong ti等）
        chinese_font_keywords = ['song', 'fangsong', 'ming', 'kai', 'hei', 'noto', 'wenquan', 'source']
        for font_name in available_fonts_list:
            font_lower = font_name.lower()
            if any(keyword in font_lower for keyword in chinese_font_keywords):
                # 优先添加中文字体到列表前面
                font_candidates.insert(0, (font_name, size, weight))
        
        # 如果没有找到中文字体，尝试使用系统默认字体（可能支持中文）
        if not font_candidates:
            # 直接尝试使用系统中实际存在的字体
            if 'song ti' in available_fonts_lower:
                font_candidates.append(('song ti', size, weight))
            elif 'fangsong ti' in available_fonts_lower:
                font_candidates.append(('fangsong ti', size, weight))
        
        # 添加标准字体候选（作为备选）
        font_candidates.extend([
            # 中文字体（Windows）
            ('Microsoft YaHei UI', size, weight),
            ('Microsoft YaHei', size, weight),
            ('SimHei', size, weight),
            # 中文字体（Linux）- 尝试多个变体
            ('WenQuanYi Micro Hei', size, weight),
            ('WenQuanYi Zen Hei', size, weight),
            ('Noto Sans CJK SC', size, weight),
            ('Noto Sans CJK', size, weight),
            ('Source Han Sans CN', size, weight),
            ('Droid Sans Fallback', size, weight),
            # 通用字体（跨平台）
            ('DejaVu Sans', size, weight),
            ('Liberation Sans', size, weight),
            ('Ubuntu', size, weight),
        ])
        
        # 尝试找到可用的字体 - 直接尝试创建，最可靠的方法
        for font_family, font_size, font_weight in font_candidates:
            try:
                # 直接尝试创建字体对象
                test_font = tkfont.Font(family=font_family, size=font_size, weight=font_weight)
                # 如果能创建成功，说明字体可用
                return (font_family, font_size, font_weight)
            except Exception:
                continue
        
        # 如果都不可用，使用系统默认字体（这总是可用）
        return ('TkDefaultFont', size, weight)
    
    def get_all_available_fonts(self) -> List[str]:
        """获取系统中所有可用的字体列表"""
        try:
            fonts = list(tkfont.families())
            # 排序并过滤掉一些无意义的字体
            fonts = sorted([f for f in fonts if not f.startswith('@')])
            return fonts
        except:
            return []
    
    def get_available_themes(self) -> List[str]:
        """获取系统中所有可用的ttk主题列表"""
        try:
            style = ttk.Style()
            themes = style.theme_names()
            return sorted(list(themes))
        except:
            return []
    
    def show_settings(self):
        """显示主题和字体设置对话框"""
        settings_window = tk.Toplevel(self.root)
        settings_window.title("主题和字体设置")
        settings_window.geometry("650x600")
        settings_window.configure(bg=self.colors['bg_section'])
        settings_window.resizable(True, True)
        settings_window.minsize(600, 550)
        
        # 居中显示
        settings_window.transient(self.root)
        settings_window.grab_set()
        
        # 主容器 - 使用grid布局以便更好地控制
        main_container = tk.Frame(settings_window, bg=self.colors['bg_section'], padx=30, pady=25)
        main_container.pack(fill=tk.BOTH, expand=True)
        
        # 创建内容容器和按钮容器的父容器
        content_wrapper = tk.Frame(main_container, bg=self.colors['bg_section'])
        content_wrapper.pack(fill=tk.BOTH, expand=True)
        
        # 按钮容器（固定高度，不扩展）
        button_wrapper = tk.Frame(main_container, bg=self.colors['bg_section'], height=60)
        button_wrapper.pack(fill=tk.X, side=tk.BOTTOM, pady=(10, 0))
        button_wrapper.pack_propagate(False)
        
        # 标题
        title_label = tk.Label(content_wrapper, 
                              text="⚙️ 主题和字体设置", 
                              font=self.font_title_obj,
                              bg=self.colors['bg_section'],
                              fg=self.colors['fg_text'])
        title_label.pack(pady=(0, 20))
        
        # 分隔线
        separator = tk.Frame(content_wrapper, height=1, bg=self.colors['border'])
        separator.pack(fill=tk.X, pady=(0, 20))
        
        # 主题选择区域
        theme_frame = tk.LabelFrame(content_wrapper, 
                                    text="选择主题",
                                    font=self.font_large_obj,
                                    bg=self.colors['bg_section'],
                                    fg=self.colors['fg_label'],
                                    padx=15, pady=15)
        theme_frame.pack(fill=tk.X, pady=(0, 15))
        
        themes = self.get_available_themes()
        if not themes:
            tk.Label(theme_frame, 
                    text="未找到可用主题",
                    font=self.font_normal_obj,
                    bg=self.colors['bg_section'],
                    fg=self.colors['fg_label']).pack()
        else:
            # 创建主题变量
            theme_var = tk.StringVar(value=self.custom_theme or (themes[0] if 'clam' not in themes else 'clam'))
            
            # 当前主题显示
            current_theme_label = tk.Label(theme_frame,
                                          text=f"当前主题: {theme_var.get()}",
                                          font=self.font_small_obj,
                                          bg=self.colors['bg_section'],
                                          fg=self.colors['fg_label'],
                                          anchor='w')
            current_theme_label.pack(fill=tk.X, pady=(0, 10))
            
            # 创建滚动框架用于主题列表
            theme_list_frame = tk.Frame(theme_frame, bg=self.colors['bg_section'])
            theme_list_frame.pack(fill=tk.BOTH, expand=True)
            
            theme_scrollbar = ttk.Scrollbar(theme_list_frame, orient="vertical")
            theme_listbox = tk.Listbox(theme_list_frame,
                                      font=self.font_normal_obj,
                                      yscrollcommand=theme_scrollbar.set,
                                      selectmode=tk.SINGLE,
                                      height=6)
            theme_scrollbar.config(command=theme_listbox.yview)
            theme_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
            theme_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
            
            # 填充主题列表
            for i, theme in enumerate(themes):
                theme_listbox.insert(tk.END, theme)
                if theme == theme_var.get():
                    theme_listbox.selection_set(i)
            
            # 主题选择事件
            def on_theme_select(event):
                selection = theme_listbox.curselection()
                if selection:
                    selected_theme = themes[selection[0]]
                    theme_var.set(selected_theme)
                    current_theme_label.config(text=f"当前主题: {selected_theme}")
            
            theme_listbox.bind('<<ListboxSelect>>', on_theme_select)
        
        # 字体选择区域
        font_frame = tk.LabelFrame(content_wrapper,
                                  text="选择字体",
                                  font=self.font_large_obj,
                                  bg=self.colors['bg_section'],
                                  fg=self.colors['fg_label'],
                                  padx=15, pady=15)
        font_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 0))
        
        fonts = self.get_all_available_fonts()
        if not fonts:
            tk.Label(font_frame,
                    text="未找到可用字体",
                    font=self.font_normal_obj,
                    bg=self.colors['bg_section'],
                    fg=self.colors['fg_label']).pack()
            font_var = tk.StringVar(value="")
        else:
            # 创建字体变量，默认使用当前字体
            current_font = self.custom_font_family or self.font_normal[0]
            if current_font not in fonts:
                current_font = fonts[0] if fonts else "TkDefaultFont"
            font_var = tk.StringVar(value=current_font)
            
            # 当前字体显示和预览
            current_font_label = tk.Label(font_frame,
                                         text=f"当前字体: {font_var.get()}",
                                         font=self.font_small_obj,
                                         bg=self.colors['bg_section'],
                                         fg=self.colors['fg_label'],
                                         anchor='w')
            current_font_label.pack(fill=tk.X, pady=(0, 5))
            
            # 字体预览
            preview_label = tk.Label(font_frame,
                                    text="这是字体预览效果：你好世界 Hello World 123",
                                    font=self.font_normal_obj,
                                    bg='white',
                                    fg=self.colors['fg_text'],
                                    relief='sunken',
                                    bd=1,
                                    padx=10,
                                    pady=8,
                                    anchor='w')
            preview_label.pack(fill=tk.X, pady=(0, 10))
            
            # 创建搜索框
            search_frame = tk.Frame(font_frame, bg=self.colors['bg_section'])
            search_frame.pack(fill=tk.X, pady=(0, 10))
            
            tk.Label(search_frame,
                    text="搜索字体:",
                    font=self.font_small_obj,
                    bg=self.colors['bg_section'],
                    fg=self.colors['fg_label']).pack(side=tk.LEFT, padx=(0, 5))
            
            search_var = tk.StringVar()
            search_entry = ttk.Entry(search_frame, textvariable=search_var, font=self.font_normal_obj)
            search_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
            
            def update_font_list(*args):
                """根据搜索关键词过滤字体列表"""
                keyword = search_var.get().lower()
                filtered_fonts = [f for f in fonts if keyword in f.lower()]
                font_listbox.delete(0, tk.END)
                for font in filtered_fonts:
                    font_listbox.insert(tk.END, font)
                    if font == font_var.get():
                        font_listbox.selection_set(tk.END)
            
            search_var.trace('w', update_font_list)
            
            # 创建滚动框架用于字体列表
            font_list_frame = tk.Frame(font_frame, bg=self.colors['bg_section'])
            font_list_frame.pack(fill=tk.BOTH, expand=True)
            
            font_scrollbar = ttk.Scrollbar(font_list_frame, orient="vertical")
            font_listbox = tk.Listbox(font_list_frame,
                                     font=self.font_small_obj,
                                     yscrollcommand=font_scrollbar.set,
                                     selectmode=tk.SINGLE,
                                     height=5)  # 减少高度，确保按钮可见
            font_scrollbar.config(command=font_listbox.yview)
            font_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
            font_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
            
            # 填充字体列表
            for i, font in enumerate(fonts):
                font_listbox.insert(tk.END, font)
                if font == font_var.get():
                    font_listbox.selection_set(i)
                    font_listbox.see(i)
            
            # 字体选择事件
            def on_font_select(event):
                selection = font_listbox.curselection()
                if selection:
                    selected_font = font_listbox.get(selection[0])
                    font_var.set(selected_font)
                    current_font_label.config(text=f"当前字体: {selected_font}")
                    # 更新预览
                    try:
                        preview_font = tkfont.Font(family=selected_font, size=10)
                        preview_label.config(font=preview_font)
                    except:
                        pass
            
            font_listbox.bind('<<ListboxSelect>>', on_font_select)
        
        # 按钮区域 - 确保始终可见（在button_wrapper中）
        button_frame = tk.Frame(button_wrapper, bg=self.colors['bg_section'])
        button_frame.pack(fill=tk.X, pady=10)
        
        def apply_settings():
            """应用设置"""
            try:
                # 获取选中的主题
                selected_theme = None
                if themes:
                    selection = theme_listbox.curselection()
                    if selection:
                        selected_theme = themes[selection[0]]
                
                # 获取选中的字体
                selected_font = None
                if fonts:
                    selection = font_listbox.curselection()
                    if selection:
                        selected_font = font_listbox.get(selection[0])
                
                # 应用设置
                if selected_theme:
                    self.custom_theme = selected_theme
                    style = ttk.Style()
                    style.theme_use(selected_theme)
                
                if selected_font:
                    self.custom_font_family = selected_font
                
                # 重新设置样式和UI
                self.setup_styles()
                self.recreate_ui()
                
                messagebox.showinfo("成功", "设置已应用！界面已更新。")
                settings_window.destroy()
            except Exception as e:
                messagebox.showerror("错误", f"应用设置失败: {str(e)}")
        
        def reset_settings():
            """重置为默认设置"""
            self.custom_theme = None
            self.custom_font_family = None
            self.setup_styles()
            self.recreate_ui()
            messagebox.showinfo("成功", "已重置为默认设置！")
            settings_window.destroy()
        
        cancel_button = ttk.Button(button_frame, text="取消",
                                  command=settings_window.destroy,
                                  style='Action.TButton')
        cancel_button.pack(side=tk.RIGHT, padx=(10, 0))
        
        reset_button = ttk.Button(button_frame, text="重置默认",
                                 command=reset_settings,
                                 style='Action.TButton')
        reset_button.pack(side=tk.RIGHT, padx=(10, 0))
        
        apply_button = ttk.Button(button_frame, text="应用",
                                 command=apply_settings,
                                 style='Primary.TButton')
        apply_button.pack(side=tk.RIGHT)
        
        # 绑定ESC键关闭
        settings_window.bind('<Escape>', lambda e: settings_window.destroy())
        
        # 设置焦点到搜索框（如果有）
        if fonts and 'search_entry' in locals():
            search_entry.focus_set()
    
    def create_ui(self):
        """创建用户界面"""
        # 设置主窗口背景色
        self.root.configure(bg=self.colors['bg_main'])
        
        # 顶部菜单栏
        if not hasattr(self, 'menubar_created'):
            # Menu组件使用Font对象
            menubar = tk.Menu(self.root, font=self.menu_font_obj)
            self.root.config(menu=menubar, bg=self.colors['bg_main'])
            
            file_menu = tk.Menu(menubar, tearoff=0, font=self.menu_font_obj)
            menubar.add_cascade(label="文件", menu=file_menu)
            file_menu.add_command(label="打开配置文件...", command=self.load_config_file)
            file_menu.add_command(label="保存配置文件", command=self.save_config)
            file_menu.add_command(label="另存为...", command=self.save_config_as)
            file_menu.add_separator()
            file_menu.add_command(label="退出", command=self.root.quit)
            
            settings_menu = tk.Menu(menubar, tearoff=0, font=self.menu_font_obj)
            menubar.add_cascade(label="设置", menu=settings_menu)
            settings_menu.add_command(label="主题和字体设置...", command=self.show_settings)
            
            help_menu = tk.Menu(menubar, tearoff=0, font=self.menu_font_obj)
            menubar.add_cascade(label="帮助", menu=help_menu)
            help_menu.add_command(label="关于", command=self.show_about)
            
            self.menubar_created = True
        
        # 主容器
        if not hasattr(self, 'main_frame'):
            self.main_frame = tk.Frame(self.root, bg=self.colors['bg_main'])
            self.main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
            
            # 顶部标题栏
            title_frame = tk.Frame(self.main_frame, bg=self.colors['accent'], height=60)
            title_frame.pack(fill=tk.X, pady=(0, 15))
            title_frame.pack_propagate(False)
            
            title_label = tk.Label(title_frame, text="⚙️ 配置文件编辑器", 
                                  font=self.font_title_obj, bg=self.colors['accent'], 
                                  fg='white', padx=20, pady=15)
            title_label.pack(side=tk.LEFT)
            
            subtitle_label = tk.Label(title_frame, text="人形机器人控制框架", 
                                     font=self.font_small_obj, bg=self.colors['accent'], 
                                     fg='white', padx=20)
            subtitle_label.pack(side=tk.LEFT, anchor='s', pady=(0, 10))
            
            # 鼓励话语显示区域（卡片样式）
            encouragement_frame = tk.Frame(self.main_frame, bg=self.colors['bg_main'])
            encouragement_frame.pack(fill=tk.X, pady=(0, 12))
            
            encouragement_card = tk.Frame(encouragement_frame, 
                                         bg='#FFF9E6',  # 温暖的米黄色背景
                                         relief='flat', bd=1, 
                                         highlightbackground='#FFD700',  # 金色边框
                                         highlightthickness=2)
            encouragement_card.pack(fill=tk.X, padx=0, pady=0)
            
            # 创建一个内部容器用于内容
            encouragement_content = tk.Frame(encouragement_card, bg='#FFF9E6')
            encouragement_content.pack(fill=tk.X, padx=20, pady=12)
            
            # 随机选择一条鼓励话语
            icon, message = random.choice(self.encouraging_messages)
            self.encouragement_icon_label = tk.Label(encouragement_content, 
                                                     text=icon,
                                                     font=('Arial', 24),
                                                     bg='#FFF9E6',
                                                     fg='#FF6B35')
            self.encouragement_icon_label.pack(side=tk.LEFT, padx=(0, 12))
            
            self.encouragement_text_label = tk.Label(encouragement_content,
                                                     text=message,
                                                     font=self.font_normal_obj,
                                                     bg='#FFF9E6',
                                                     fg='#2C3E50',
                                                     anchor='w')
            self.encouragement_text_label.pack(side=tk.LEFT, fill=tk.X, expand=True)
            
            # 添加一个刷新按钮（小图标）
            refresh_btn = tk.Label(encouragement_content,
                                  text="🔄",
                                  font=('Arial', 14),
                                  bg='#FFF9E6',
                                  fg='#666666',
                                  cursor='hand2')
            refresh_btn.pack(side=tk.RIGHT, padx=(10, 0))
            refresh_btn.bind('<Button-1>', lambda e: self.refresh_encouragement())
            refresh_btn.bind('<Enter>', lambda e: refresh_btn.config(fg='#3498DB'))
            refresh_btn.bind('<Leave>', lambda e: refresh_btn.config(fg='#666666'))
            
            # 保存引用以便后续更新
            self.encouragement_card = encouragement_card
            self.encouragement_content = encouragement_content
            
            # 启动自动更新定时器（每10秒更新一次）
            self.start_encouragement_timer()
            
            # 顶部工具栏（卡片样式）
            toolbar_frame = tk.Frame(self.main_frame, bg=self.colors['bg_main'])
            toolbar_frame.pack(fill=tk.X, pady=(0, 15))
            
            toolbar = tk.Frame(toolbar_frame, bg=self.colors['bg_section'], 
                              relief='flat', bd=1, highlightbackground=self.colors['border'],
                              highlightthickness=1)
            toolbar.pack(fill=tk.X, padx=0, pady=0)
            
            # 按钮容器
            button_frame = tk.Frame(toolbar, bg=self.colors['bg_section'])
            button_frame.pack(side=tk.LEFT, padx=15, pady=12)
            
            # ttk.Button会自动使用样式中的字体
            btn_load = ttk.Button(button_frame, text="📂 加载配置", command=self.load_config,
                                 style='Primary.TButton')
            btn_load.pack(side=tk.LEFT, padx=(0, 8))
            
            btn_generate = ttk.Button(button_frame, text="💾 生成配置文件", 
                                     command=self.generate_config, style='Primary.TButton')
            btn_generate.pack(side=tk.LEFT, padx=(0, 8))
            
            btn_preview = ttk.Button(button_frame, text="👁️ 预览配置", 
                                    command=self.preview_config, style='Action.TButton')
            btn_preview.pack(side=tk.LEFT, padx=(0, 15))
            
            # 文件路径显示（可点击选择文件）
            file_info_frame = tk.Frame(toolbar, bg=self.colors['bg_section'])
            file_info_frame.pack(side=tk.RIGHT, padx=15, pady=12)
            
            file_label_title = tk.Label(file_info_frame, text="当前文件:", 
                                       font=self.font_small_obj, bg=self.colors['bg_section'],
                                       fg=self.colors['fg_label'])
            file_label_title.pack(side=tk.LEFT, padx=(0, 5))
            
            # 文件路径标签 - 可点击，鼠标悬停时显示手型光标
            display_path = self.get_display_path(self.config_file)
            self.file_label = tk.Label(file_info_frame, text=display_path, 
                                      font=self.font_small_obj, bg=self.colors['bg_section'],
                                      fg=self.colors['accent'], anchor='w',
                                      cursor='hand2')  # 鼠标悬停时显示手型光标
            self.file_label.pack(side=tk.LEFT)
            
            # 绑定点击事件
            self.file_label.bind('<Button-1>', self.on_file_label_click)
            self.file_label.bind('<Enter>', lambda e: self.file_label.config(fg=self.colors['bg_button_hover']))
            self.file_label.bind('<Leave>', lambda e: self.file_label.config(fg=self.colors['accent']))
            
            # 创建Notebook（标签页）- 使用卡片样式
            notebook_frame = tk.Frame(self.main_frame, bg=self.colors['bg_main'])
            notebook_frame.pack(fill=tk.BOTH, expand=True)
            
            self.notebook = ttk.Notebook(notebook_frame)
            self.notebook.pack(fill=tk.BOTH, expand=True)
        
        # 为每个配置部分创建标签页
        self.create_notebook_tabs()
    
    def create_notebook_tabs(self):
        """创建或重新创建Notebook标签页"""
        # 清除现有标签页
        for i in range(self.notebook.index("end") - 1, -1, -1):
            self.notebook.forget(i)
        
        # 清空widgets
        self.widgets = {}
        
        # 为每个配置部分创建标签页
        for section in self.all_sections:
            frame = ttk.Frame(self.notebook, padding="10")
            self.notebook.add(frame, text=self.get_section_display_name(section))
            self.create_section_ui(frame, section)
    
    def get_display_path(self, file_path: str) -> str:
        """获取文件路径的显示文本（如果太长则截断）"""
        if len(file_path) > 60:
            return "..." + file_path[-57:]
        return file_path
    
    def recreate_ui(self):
        """重新创建UI（用于加载新配置后或应用新设置）"""
        self.create_notebook_tabs()
        # 更新文件路径显示
        if hasattr(self, 'file_label'):
            display_path = self.get_display_path(self.config_file)
            self.file_label.config(text=display_path, font=self.font_small_obj)
        
        # 更新标题栏和其他UI元素的字体
        if hasattr(self, 'main_frame'):
            self._update_ui_fonts()
    
    def _update_ui_fonts(self):
        """更新UI中所有文本元素的字体（用于应用新设置后）"""
        # 这个方法可以递归更新所有子widget的字体
        # 但为了简单起见，我们只更新关键的可见元素
        try:
            # 更新鼓励话语标签的字体
            if hasattr(self, 'encouragement_text_label'):
                self.encouragement_text_label.config(font=self.font_normal_obj)
            
            # 更新文件标签字体
            if hasattr(self, 'file_label'):
                self.file_label.config(font=self.font_small_obj)
        except:
            pass  # 如果某些元素不存在，忽略错误
    
    def refresh_encouragement(self):
        """刷新鼓励话语"""
        icon, message = random.choice(self.encouraging_messages)
        self.encouragement_icon_label.config(text=icon)
        self.encouragement_text_label.config(text=message)
    
    def start_encouragement_timer(self):
        """启动鼓励话语自动更新定时器（每10秒更新一次）"""
        # 10秒 = 10000毫秒
        self.root.after(2000, self.auto_refresh_encouragement)
    
    def auto_refresh_encouragement(self):
        """自动刷新鼓励话语（定时器回调）"""
        self.refresh_encouragement()
        # 继续设置下一次定时器
        self.start_encouragement_timer()
    
    def get_section_display_name(self, section: str) -> str:
        """获取配置部分的显示名称"""
        names = {
            "motor_communication": "通信配置",
            "simulation": "仿真配置",
            "robot_control_parameters": "机器人控制参数",
            "logging": "日志配置",
            "gamepad": "游戏手柄/遥控器",
            "algorithm_launcher": "算法启动器",
            "imu": "IMU配置",
            "safety_checker": "安全检查器",
            "rl_walk": "RL步行状态机",
            "rl_dance": "RL舞蹈状态机",
            "rl_hybrid": "RL混合状态机",
            "recovery_stand": "恢复站立状态机",
            "development": "开发模式状态机",
        }
        return names.get(section, section)
    
    def create_section_ui(self, parent: ttk.Frame, section: str):
        """为特定配置部分创建UI"""
        # 创建滚动框架
        canvas = tk.Canvas(parent, bg=self.colors['bg_section'], highlightthickness=0)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        
        # 加载并设置背景图片（带透明度）
        bg_image = self.load_background_image(canvas)
        if bg_image:
            # 将背景图片添加到canvas
            canvas.create_image(0, 0, anchor="nw", image=bg_image, tags="bg_image")
            canvas.lower("bg_image")  # 将背景图片放到最底层
            # Canvas背景保持默认，让背景图片显示
        
        # Frame使用与Canvas相同的背景色，让背景图片能透过（使用系统默认背景色）
        scrollable_frame = tk.Frame(canvas, padx=20, pady=20)  # 不设置bg，使用系统默认
        
        def update_scroll_region(event):
            canvas.configure(scrollregion=canvas.bbox("all"))
            # 更新背景图片位置
            if bg_image:
                canvas.coords("bg_image", 0, 0)
        
        scrollable_frame.bind("<Configure>", update_scroll_region)
        
        canvas_window = canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        
        def configure_canvas_width(event):
            canvas_width = event.width
            canvas.itemconfig(canvas_window, width=canvas_width)
            # 重新缩放背景图片以适应canvas大小
            if bg_image and PIL_AVAILABLE:
                self.update_background_image(canvas, bg_image, event.width, event.height)
        
        canvas.bind("<Configure>", configure_canvas_width)
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # 存储该部分的widgets
        self.widgets[section] = {}
        
        # 获取配置数据
        section_data = self.config_data.get(section, {})
        
        # 如果数据为空，显示提示信息
        if not section_data:
            empty_frame = tk.Frame(scrollable_frame, bg=self.colors['bg_section'])
            empty_frame.pack(fill=tk.BOTH, expand=True, pady=50)
            
            icon_label = tk.Label(empty_frame, text="📋", font=('Arial', 48), 
                                 bg=self.colors['bg_section'], fg=self.colors['border'])
            icon_label.pack(pady=(0, 15))
            
            label = tk.Label(empty_frame, 
                           text="该配置部分为空\n请先加载配置文件或在其他部分编辑后生成配置", 
                           font=self.font_normal_obj, bg=self.colors['bg_section'],
                           fg=self.colors['fg_label'], justify=tk.CENTER)
            label.pack()
        else:
            # 创建输入字段
            self.create_fields(scrollable_frame, section, section_data, "")
        
        # 更新滚动区域
        canvas.update_idletasks()
        canvas.configure(scrollregion=canvas.bbox("all"))
    
    def load_background_image(self, canvas: tk.Canvas, opacity: float = 0.15):
        """加载背景图片并设置透明度
        
        Args:
            canvas: Canvas对象
            opacity: 透明度 (0.0-1.0)，值越小越透明，默认0.15（15%不透明度）
        
        Returns:
            PhotoImage对象或None
        """
        if not PIL_AVAILABLE:
            return None
        
        try:
            # 获取图片路径
            script_dir = os.path.dirname(os.path.abspath(__file__))
            project_root = os.path.dirname(script_dir)
            image_path = os.path.join(project_root, "actor_model", "e3.png")
            
            if not os.path.exists(image_path):
                return None
            
            # 打开图片
            img = Image.open(image_path)
            
            # 转换为RGBA模式（支持透明度）
            if img.mode != 'RGBA':
                img = img.convert('RGBA')
            
            # 调整透明度
            alpha = img.split()[3]  # 获取alpha通道
            alpha = ImageEnhance.Brightness(alpha).enhance(opacity)  # 降低alpha值
            img.putalpha(alpha)  # 设置新的alpha通道
            
            # 转换为PhotoImage
            photo = ImageTk.PhotoImage(img)
            
            # 保存引用（防止垃圾回收）
            if not hasattr(self, '_bg_images'):
                self._bg_images = []
            self._bg_images.append(photo)
            
            return photo
        except Exception as e:
            print(f"[警告] 无法加载背景图片: {e}")
            return None
    
    def update_background_image(self, canvas: tk.Canvas, photo_image, width: int, height: int):
        """更新背景图片大小以适应canvas"""
        try:
            # 这里可以重新缩放图片，但为了性能，暂时不缩放
            # 如果图片很大，可以考虑缓存缩放后的版本
            pass
        except Exception:
            pass
    
    def get_choice_options(self, section: str, full_key: str) -> Optional[List[str]]:
        """获取字段的可选值列表，如果该字段支持下拉选择"""
        # motor_communication.type: "lcm" 或 "ethercat"
        if section == "motor_communication" and full_key == "type":
            return ["lcm", "ethercat"]
        # gamepad.device_type: "gamepad" 或 "at9s"
        if section == "gamepad" and full_key == "device_type":
            return ["gamepad", "at9s"]
        return None
    
    def create_fields(self, parent: ttk.Frame, section: str, data: Dict[str, Any], prefix: str = "", indent: int = 0):
        """递归创建配置字段"""
        # 获取父容器的背景色（用于透明效果）
        try:
            parent_bg = parent.cget('bg') if isinstance(parent, tk.Widget) else self.colors['bg_section']
        except:
            parent_bg = self.colors['bg_section']
        
        row = 0
        
        for key, value in data.items():
            full_key = f"{prefix}.{key}" if prefix else key
            
            if isinstance(value, dict):
                # 嵌套字典：创建子框架（卡片样式，半透明背景）
                subframe = tk.LabelFrame(parent, text=f"  {key}  ", 
                                        font=self.font_large_obj, 
                                        bg='#F8F9FA',  # 浅灰白色背景（模拟半透明效果）
                                        fg=self.colors['fg_label'],
                                        relief='flat', bd=1,
                                        highlightbackground=self.colors['border'],
                                        highlightthickness=1,
                                        padx=10, pady=8)
                subframe.grid(row=row, column=0, columnspan=2, sticky="ew", 
                            pady=8, padx=(indent * 20, 0))
                parent.columnconfigure(0, weight=1)
                
                self.create_fields(subframe, section, value, full_key, indent + 1)
                row += 1
            elif isinstance(value, list):
                # 列表：创建多行文本输入
                label = tk.Label(parent, text=f"{key}:", font=self.font_normal_obj,
                               bg=parent_bg, fg=self.colors['fg_label'],
                               anchor='w')
                label.grid(row=row, column=0, sticky="w", padx=(indent * 20, 15), pady=6)
                
                # 创建文本框和按钮
                list_frame = ttk.Frame(parent)
                list_frame.grid(row=row, column=1, sticky="ew", pady=2)
                
                text_widget = tk.Text(list_frame, height=4, width=50, font=self.font_normal_obj)
                text_widget.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
                text_widget.insert("1.0", self.format_list_value(value))
                
                self.widgets[section][full_key] = ("list", text_widget)
                row += 1
            elif isinstance(value, bool):
                # 布尔值：创建复选框
                label = tk.Label(parent, text=f"{key}:", font=self.font_normal_obj,
                               bg=parent_bg, fg=self.colors['fg_label'],
                               anchor='w')
                label.grid(row=row, column=0, sticky="w", padx=(indent * 20, 15), pady=6)
                
                var = tk.BooleanVar(value=value)
                checkbox = ttk.Checkbutton(parent, variable=var, style='TCheckbutton')
                checkbox.grid(row=row, column=1, sticky="w", pady=6)
                
                self.widgets[section][full_key] = ("bool", var)
                row += 1
            else:
                # 检查是否支持下拉选择
                choice_options = self.get_choice_options(section, full_key)
                
                label = tk.Label(parent, text=f"{key}:", font=self.font_normal_obj,
                               bg=parent_bg, fg=self.colors['fg_label'],
                               anchor='w')
                label.grid(row=row, column=0, sticky="w", padx=(indent * 20, 15), pady=6)
                
                if choice_options is not None:
                    # 使用下拉选择框
                    var = tk.StringVar(value=str(value))
                    combobox = ttk.Combobox(parent, textvariable=var, values=choice_options, 
                                           state="readonly", width=47, font=self.font_normal_obj)
                    combobox.grid(row=row, column=1, sticky="ew", pady=6, padx=(0, 10))
                    
                    self.widgets[section][full_key] = ("choice", var)
                else:
                    # 普通值：创建输入框
                    entry = ttk.Entry(parent, width=50, font=self.font_normal_obj)
                    entry.grid(row=row, column=1, sticky="ew", pady=6, padx=(0, 10))
                    entry.insert(0, str(value))
                    
                    self.widgets[section][full_key] = ("value", entry)
                row += 1
        
        parent.columnconfigure(1, weight=1)
    
    def format_list_value(self, value: List) -> str:
        """格式化列表值为字符串"""
        if isinstance(value[0], (int, float)) if value else True:
            return "[" + ", ".join(str(v) for v in value) + "]"
        else:
            return "\n".join(f"- {v}" for v in value)
    
    def parse_list_value(self, text: str) -> List:
        """解析文本为列表"""
        text = text.strip()
        if not text:
            return []
        
        # 尝试解析为Python列表格式 [1, 2, 3]
        if text.startswith("[") and text.endswith("]"):
            try:
                # 安全评估
                text = text.strip("[]")
                items = [item.strip() for item in text.split(",") if item.strip()]
                result = []
                for item in items:
                    # 尝试转换为数字
                    try:
                        if "." in item:
                            result.append(float(item))
                        else:
                            result.append(int(item))
                    except ValueError:
                        # 保留为字符串，去除引号
                        result.append(item.strip('"\''))
                return result
            except:
                pass
        
        # 解析为YAML列表格式
        lines = text.split("\n")
        result = []
        for line in lines:
            line = line.strip()
            if line.startswith("-"):
                item = line[1:].strip()
                # 尝试转换为数字
                try:
                    if "." in item:
                        result.append(float(item))
                    else:
                        result.append(int(item))
                except ValueError:
                    result.append(item.strip('"\''))
            elif line:
                # 直接的行
                try:
                    if "." in line:
                        result.append(float(line))
                    else:
                        result.append(int(line))
                except ValueError:
                    result.append(line.strip('"\''))
        return result
    
    def populate_ui(self):
        """从配置数据填充UI"""
        for section in self.all_sections:
            section_data = self.config_data.get(section, {})
            self.update_widgets_from_data(section, section_data, "")
    
    def update_widgets_from_data(self, section: str, data: Dict[str, Any], prefix: str = ""):
        """递归更新widgets的值"""
        for key, value in data.items():
            full_key = f"{prefix}.{key}" if prefix else key
            
            if isinstance(value, dict):
                self.update_widgets_from_data(section, value, full_key)
            elif full_key in self.widgets.get(section, {}):
                widget_type, widget = self.widgets[section][full_key]
                
                if widget_type == "list":
                    widget.delete("1.0", tk.END)
                    widget.insert("1.0", self.format_list_value(value))
                elif widget_type == "bool":
                    widget.set(value)
                elif widget_type == "choice":
                    # 下拉选择框
                    widget.set(str(value))
                else:
                    widget.delete(0, tk.END)
                    widget.insert(0, str(value))
    
    def on_file_label_click(self, event=None):
        """文件标签点击事件 - 打开文件选择对话框"""
        self.load_config_file()
    
    def load_config_file(self):
        """从文件加载配置"""
        # 获取默认目录（当前配置文件所在目录，或项目根目录）
        if os.path.exists(self.config_file):
            default_dir = os.path.dirname(os.path.abspath(self.config_file))
        else:
            # 使用项目根目录作为默认路径
            script_dir = os.path.dirname(os.path.abspath(__file__))
            project_root = os.path.dirname(script_dir)
            default_dir = project_root
        
        file_path = filedialog.askopenfilename(
            title="选择配置文件",
            initialdir=default_dir,  # 设置默认目录为当前文件夹
            filetypes=[("YAML files", "*.yaml *.yml"), ("All files", "*.*")]
        )
        if file_path:
            self.config_file = file_path
            self.load_config()
    
    def load_config(self):
        """加载配置文件"""
        try:
            if not os.path.exists(self.config_file):
                messagebox.showerror("错误", f"配置文件不存在: {self.config_file}")
                return
            
            with open(self.config_file, 'r', encoding='utf-8') as f:
                self.config_data = yaml.safe_load(f) or {}
            
            # 确保所有部分都存在
            for section in self.all_sections:
                if section not in self.config_data:
                    self.config_data[section] = {}
            
            # 重新创建所有UI（因为配置可能完全不同）
            self.recreate_ui()
            
            # 更新文件路径显示
            display_path = self.get_display_path(self.config_file)
            self.file_label.config(text=display_path)
            
            messagebox.showinfo("成功", f"配置文件已加载: {os.path.basename(self.config_file)}")
        except Exception as e:
            messagebox.showerror("错误", f"加载配置文件失败: {str(e)}")
            import traceback
            traceback.print_exc()
    
    def save_config(self):
        """保存配置文件"""
        self.generate_config()
    
    def save_config_as(self):
        """另存为配置文件"""
        # 获取默认目录
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(script_dir)
        default_dir = project_root
        
        # 如果当前文件路径有效，使用其目录
        if self.config_file and os.path.dirname(self.config_file):
            try:
                default_dir = os.path.dirname(os.path.abspath(self.config_file))
            except:
                pass
        
        file_path = filedialog.asksaveasfilename(
            title="另存为配置文件",
            initialdir=default_dir,
            defaultextension=".yaml",
            initialfile=os.path.basename(self.config_file) if self.config_file else "config.yaml",
            filetypes=[("YAML files", "*.yaml *.yml"), ("All files", "*.*")]
        )
        if file_path:
            self.generate_config(file_path)
    
    def get_widget_value(self, section: str, key: str):
        """从widget获取值"""
        if section not in self.widgets or key not in self.widgets[section]:
            return None
        
        widget_type, widget = self.widgets[section][key]
        
        if widget_type == "list":
            text = widget.get("1.0", tk.END).strip()
            return self.parse_list_value(text)
        elif widget_type == "bool":
            return widget.get()
        elif widget_type == "choice":
            # 下拉选择框：直接返回字符串值
            return widget.get()
        else:
            value_str = widget.get().strip()
            # 尝试转换为数字
            try:
                if "." in value_str:
                    return float(value_str)
                else:
                    return int(value_str)
            except ValueError:
                # 返回字符串，去除引号
                return value_str.strip('"\'')
    
    def collect_config_from_ui(self) -> Dict[str, Any]:
        """从UI收集配置数据"""
        config = {}
        
        for section in self.all_sections:
            section_config = {}
            
            if section in self.widgets:
                for full_key, (widget_type, widget) in self.widgets[section].items():
                    value = self.get_widget_value(section, full_key)
                    self.set_nested_value(section_config, full_key, value)
            
            config[section] = section_config
        
        return config
    
    def set_nested_value(self, config: Dict[str, Any], key_path: str, value: Any):
        """设置嵌套字典的值"""
        keys = key_path.split(".")
        current = config
        
        for key in keys[:-1]:
            if key not in current:
                current[key] = {}
            current = current[key]
        
        current[keys[-1]] = value
    
    def generate_config(self, file_path: str = None):
        """生成配置文件
        
        Args:
            file_path: 保存的文件路径，如果为None则使用当前文件路径或弹出对话框选择
        """
        try:
            # 如果没有指定文件路径，检查当前文件是否存在
            if file_path is None:
                if os.path.exists(self.config_file):
                    # 文件存在，直接保存到当前文件
                    file_path = self.config_file
                else:
                    # 文件不存在，弹出对话框让用户选择保存位置
                    script_dir = os.path.dirname(os.path.abspath(__file__))
                    project_root = os.path.dirname(script_dir)
                    default_dir = project_root
                    
                    # 如果当前文件路径有效，使用其目录
                    if self.config_file and os.path.dirname(self.config_file):
                        try:
                            default_dir = os.path.dirname(os.path.abspath(self.config_file))
                        except:
                            pass
                    
                    file_path = filedialog.asksaveasfilename(
                        title="保存配置文件",
                        initialdir=default_dir,
                        defaultextension=".yaml",
                        initialfile=os.path.basename(self.config_file) if self.config_file else "config.yaml",
                        filetypes=[("YAML files", "*.yaml *.yml"), ("All files", "*.*")]
                    )
                    
                    if not file_path:  # 用户取消了选择
                        return
            
            # 从UI收集配置
            ui_config = self.collect_config_from_ui()
            
            # 合并到现有配置（保留UI中未显示的字段）
            for section in self.all_sections:
                if section in ui_config:
                    # 深度合并
                    self.deep_merge(self.config_data.setdefault(section, {}), ui_config[section])
            
            # 按照指定顺序组织配置（状态机配置在最后）
            ordered_config = {}
            
            # 先添加非状态机配置
            for section in self.non_fsm_sections:
                if section in self.config_data:
                    ordered_config[section] = self.config_data[section]
            
            # 再添加状态机配置
            for section in self.fsm_sections:
                if section in self.config_data:
                    ordered_config[section] = self.config_data[section]
            
            # 生成YAML内容（添加文件头）
            header = f"""# 人形机器人部署通用框架 - 主配置文件
# 作者: Han Jiang (jh18954242606@163.com)
# 日期: {datetime.now().strftime('%Y-%m')}
# 生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
# 注意: 此文件由配置编辑器自动生成

"""
            yaml_content = self.generate_yaml(ordered_config)
            
            # 保存到文件
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(header)
                f.write(yaml_content)
                if not yaml_content.endswith('\n'):
                    f.write('\n')
            
            # 如果保存到新文件，更新当前文件路径
            if file_path != self.config_file:
                self.config_file = file_path
                # 更新文件路径显示
                display_path = self.get_display_path(self.config_file)
                self.file_label.config(text=display_path)
            
            messagebox.showinfo("成功", f"配置文件已生成: {os.path.basename(file_path)}")
        except Exception as e:
            messagebox.showerror("错误", f"生成配置文件失败: {str(e)}")
            import traceback
            traceback.print_exc()
    
    def deep_merge(self, base: Dict, update: Dict):
        """深度合并字典"""
        for key, value in update.items():
            if key in base and isinstance(base[key], dict) and isinstance(value, dict):
                self.deep_merge(base[key], value)
            else:
                base[key] = value
    
    def generate_yaml(self, config: Dict[str, Any], indent: int = 0) -> str:
        """生成YAML格式的配置内容"""
        if not config:
            return ""
        
        lines = []
        indent_str = "  " * indent
        items = list(config.items())
        
        for idx, (key, value) in enumerate(items):
            if isinstance(value, dict):
                if not value:
                    lines.append(f"{indent_str}{key}: {{}}")
                else:
                    lines.append(f"{indent_str}{key}:")
                    sub_yaml = self.generate_yaml(value, indent + 1)
                    if sub_yaml.strip():
                        lines.append(sub_yaml)
            elif isinstance(value, list):
                if not value:
                    lines.append(f"{indent_str}{key}: []")
                elif all(isinstance(v, (int, float)) for v in value):
                    # 数字列表：使用紧凑格式 [1, 2, 3]
                    values_str = ", ".join(str(v) for v in value)
                    lines.append(f"{indent_str}{key}: [{values_str}]")
                else:
                    # 字符串列表或其他：使用多行格式
                    lines.append(f"{indent_str}{key}:")
                    for item in value:
                        if isinstance(item, str):
                            lines.append(f'{indent_str}  - "{item}"')
                        else:
                            lines.append(f"{indent_str}  - {item}")
            elif isinstance(value, bool):
                lines.append(f"{indent_str}{key}: {str(value).lower()}")
            elif isinstance(value, (int, float)):
                lines.append(f"{indent_str}{key}: {value}")
            elif isinstance(value, str):
                # 字符串：如果包含特殊字符或为空，使用引号
                if not value or " " in value or ":" in value or value in ["True", "False", "true", "false", "None", "null"]:
                    # 转义引号
                    escaped = value.replace('"', '\\"')
                    lines.append(f'{indent_str}{key}: "{escaped}"')
                else:
                    lines.append(f"{indent_str}{key}: {value}")
            elif value is None:
                lines.append(f"{indent_str}{key}: null")
            else:
                lines.append(f"{indent_str}{key}: {value}")
        
        result = "\n".join(lines)
        # 确保末尾有换行（只在顶层）
        if indent == 0 and result:
            result += "\n"
        return result
    
    def preview_config(self):
        """预览生成的配置"""
        try:
            ui_config = self.collect_config_from_ui()
            
            # 合并配置
            for section in self.all_sections:
                if section in ui_config:
                    self.deep_merge(self.config_data.setdefault(section, {}), ui_config[section])
            
            # 按顺序组织
            ordered_config = {}
            for section in self.non_fsm_sections:
                if section in self.config_data:
                    ordered_config[section] = self.config_data[section]
            for section in self.fsm_sections:
                if section in self.config_data:
                    ordered_config[section] = self.config_data[section]
            
            yaml_content = self.generate_yaml(ordered_config)
            
            # 显示预览窗口
            preview_window = tk.Toplevel(self.root)
            preview_window.title("配置预览")
            preview_window.geometry("900x700")
            preview_window.configure(bg=self.colors['bg_main'])
            
            # 预览窗口标题
            preview_title = tk.Frame(preview_window, bg=self.colors['accent'], height=50)
            preview_title.pack(fill=tk.X)
            preview_title.pack_propagate(False)
            
            title_label = tk.Label(preview_title, text="📄 配置预览", 
                                  font=self.font_title_obj, bg=self.colors['accent'], 
                                  fg='white', padx=20, pady=12)
            title_label.pack(side=tk.LEFT)
            
            # 文本区域
            text_frame = tk.Frame(preview_window, bg=self.colors['bg_main'])
            text_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=15)
            
            text_widget = scrolledtext.ScrolledText(text_frame, wrap=tk.WORD, 
                                                   font=self.font_normal_obj,
                                                   bg='white', fg='#2C3E50',
                                                   relief='flat', bd=1,
                                                   highlightbackground=self.colors['border'],
                                                   highlightthickness=1,
                                                   padx=10, pady=10)
            text_widget.pack(fill=tk.BOTH, expand=True)
            text_widget.insert("1.0", yaml_content)
            text_widget.config(state=tk.DISABLED)
            
        except Exception as e:
            messagebox.showerror("错误", f"预览配置失败: {str(e)}")
    
    def show_about(self):
        """显示关于对话框"""
        # 创建自定义关于对话框窗口，更好的格式和间距
        about_window = tk.Toplevel(self.root)
        about_window.title("关于")
        about_window.geometry("500x420")
        about_window.configure(bg=self.colors['bg_section'])
        about_window.resizable(False, False)
        
        # 居中显示
        about_window.transient(self.root)
        about_window.grab_set()
        
        # 主容器
        main_container = tk.Frame(about_window, bg=self.colors['bg_section'], padx=30, pady=25)
        main_container.pack(fill=tk.BOTH, expand=True)
        
        # 标题
        title_label = tk.Label(main_container, 
                              text="人形机器人控制框架", 
                              font=self.font_title_obj,
                              bg=self.colors['bg_section'],
                              fg=self.colors['fg_text'])
        title_label.pack(pady=(0, 5))
        
        subtitle_label = tk.Label(main_container, 
                                 text="配置文件编辑器", 
                                 font=self.font_large_obj,
                                 bg=self.colors['bg_section'],
                                 fg=self.colors['fg_label'])
        subtitle_label.pack(pady=(0, 20))
        
        # 分隔线
        separator = tk.Frame(main_container, height=1, bg=self.colors['border'])
        separator.pack(fill=tk.X, pady=(0, 20))
        
        # 版本信息
        version_frame = tk.Frame(main_container, bg=self.colors['bg_section'])
        version_frame.pack(fill=tk.X, pady=(0, 15))
        
        tk.Label(version_frame, text="版本:", 
                font=self.font_normal_obj, bg=self.colors['bg_section'],
                fg=self.colors['fg_label'], anchor='w').pack(fill=tk.X, pady=(0, 3))
        tk.Label(version_frame, text="  1.0", 
                font=self.font_normal_obj, bg=self.colors['bg_section'],
                fg=self.colors['fg_text'], anchor='w').pack(fill=tk.X, pady=(0, 10))
        
        tk.Label(version_frame, text="作者:", 
                font=self.font_normal_obj, bg=self.colors['bg_section'],
                fg=self.colors['fg_label'], anchor='w').pack(fill=tk.X, pady=(0, 3))
        tk.Label(version_frame, text="  Han Jiang (jh18954242606@163.com)", 
                font=self.font_normal_obj, bg=self.colors['bg_section'],
                fg=self.colors['fg_text'], anchor='w').pack(fill=tk.X, pady=(0, 15))
        
        # 功能列表
        tk.Label(main_container, text="功能：", 
                font=self.font_normal_obj, bg=self.colors['bg_section'],
                fg=self.colors['fg_label'], anchor='w').pack(fill=tk.X, pady=(0, 8))
        
        features = [
            "  • 可视化编辑所有配置项",
            "  • 支持嵌套配置结构",
            "  • 一键生成配置文件",
            "  • 状态机配置自动放在最后"
        ]
        
        for feature in features:
            tk.Label(main_container, text=feature, 
                    font=self.font_normal_obj, bg=self.colors['bg_section'],
                    fg=self.colors['fg_text'], anchor='w').pack(fill=tk.X, pady=(0, 5))
        
        # 使用方法
        tk.Label(main_container, text="使用方法：", 
                font=self.font_normal_obj, bg=self.colors['bg_section'],
                fg=self.colors['fg_label'], anchor='w').pack(fill=tk.X, pady=(15, 8))
        
        methods = [
            "  1. 打开或加载现有配置文件",
            "  2. 在各个标签页中编辑配置项",
            "  3. 点击“生成配置文件”保存"
        ]
        
        for method in methods:
            tk.Label(main_container, text=method, 
                    font=self.font_normal_obj, bg=self.colors['bg_section'],
                    fg=self.colors['fg_text'], anchor='w').pack(fill=tk.X, pady=(0, 5))
        
        # 关闭按钮
        button_frame = tk.Frame(main_container, bg=self.colors['bg_section'])
        button_frame.pack(fill=tk.X, pady=(20, 0))
        
        close_button = ttk.Button(button_frame, text="确定", 
                                 command=about_window.destroy,
                                 style='Primary.TButton')
        close_button.pack()
        
        # 绑定ESC键关闭
        about_window.bind('<Escape>', lambda e: about_window.destroy())
        about_window.bind('<Return>', lambda e: about_window.destroy())
        
        # 设置焦点到关闭按钮
        close_button.focus_set()
    
    def get_default_config(self) -> Dict[str, Any]:
        """获取默认配置结构"""
        # 返回空配置结构，让用户从现有文件加载
        return {section: {} for section in self.all_sections}


def main():
    """主函数"""
    # 解析命令行参数
    config_file = None
    if len(sys.argv) > 1:
        config_file = sys.argv[1]
    else:
        # 默认使用项目根目录的config.yaml
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(script_dir)
        config_file = os.path.join(project_root, "config.yaml")
    
    root = tk.Tk()
    app = ConfigEditor(root, config_file)
    root.mainloop()


if __name__ == "__main__":
    main()

