# WhiteBoardBot

白板机器人项目，可与 ESP32 等硬件配合使用。

---

## 一、环境要求

- **Python**: 3.9 或以上
- **系统**: macOS / Linux / Windows
- **硬件**（可选）: ESP32 开发板、串口连接

---

## 二、虚拟环境

### 2.1 创建虚拟环境（已完成可跳过）

项目已包含虚拟环境目录 `.venv`。若需重新创建：

```bash
cd <repo>
python3 -m venv .venv
```

### 2.2 激活虚拟环境

**macOS / Linux:**

```bash
source .venv/bin/activate
```

**Windows (CMD):**

```cmd
.venv\Scripts\activate.bat
```

**Windows (PowerShell):**

```powershell
.venv\Scripts\Activate.ps1
```

激活后终端提示符前会出现 `(.venv)`。

### 2.3 安装依赖

在**已激活**虚拟环境的情况下：

```bash
pip install -r requirements.txt
```

### 2.4 退出虚拟环境

```bash
deactivate
```

---

## 三、使用说明

### 3.1 日常开发流程

1. 进入项目目录并激活虚拟环境：
   ```bash
   cd <repo>
   source .venv/bin/activate   # macOS/Linux
   ```

2. 确认依赖已安装：
   ```bash
   pip install -r requirements.txt
   ```

3. 运行项目主程序（根据后续添加的入口调整）：
   ```bash
   python -m src.main   # 示例，以实际入口为准
   ```

### 3.2 与 ESP32 通信（示例）

- 将 ESP32 通过 USB 连接到电脑，记下串口设备名：
  - macOS: `/dev/cu.usbserial-*` 或 `/dev/tty.usbserial-*`
  - Linux: `/dev/ttyUSB*` 或 `/dev/ttyACM*`
  - Windows: `COM3`、`COM4` 等
- 在代码或配置中设置对应串口与波特率（如 115200）后即可通信。

### 3.3 Cursor / VS Code 使用虚拟环境

1. `Cmd + Shift + P` 打开命令面板。
2. 输入并选择 **Python: Select Interpreter**。
3. 选择项目中的 `.venv` 解释器（路径包含 `WhiteBoardBot/.venv`）。

之后在 Cursor 中运行、调试都会使用该虚拟环境。

---

## 3.4 PlatformIO（ESP32 固件开发）

本项目已包含 PlatformIO 配置，用于编写、编译和烧录 ESP32 固件。

### 安装 PlatformIO

**方式一：Cursor / VS Code 扩展**

1. 打开扩展市场（`Cmd + Shift + X`）。
2. 搜索 **PlatformIO IDE**（或 **PlatformIO**）。
3. 安装后按提示重启。若 Cursor 应用商店没有，可到 [OpenVSX](https://open-vsx.org/) 下载 VSIX 手动安装。

**方式二：仅用命令行（不装扩展）**

```bash
pip install platformio
# 或
pip install -U platformio
```

之后在项目根目录用 `pio run`、`pio run -t upload` 等命令即可。

### 常用命令（在项目根目录执行）

| 命令 | 说明 |
|------|------|
| `pio run` | 编译固件 |
| `pio run -t upload` | 编译并烧录到已连接的 ESP32 |
| `pio device list` | 列出可用串口 |
| `pio run -t clean` | 清理编译产物 |
| `pio run -e esp32dev` | 指定环境编译（当前默认即 esp32dev） |

在 Cursor 中安装 PlatformIO 扩展后，底部会出现 **PlatformIO** 图标，可点击 **Build**、**Upload**、**Serial Monitor** 等按钮操作。

### Cursor 与 C/C++ 扩展说明

Cursor 与 **Microsoft C/C++ 扩展** 的许可不兼容，不能使用该扩展。PlatformIO 自带的编译/智能提示可正常使用。若需更好 C++ 补全，可配合 **clangd**，但需自行配置 include 路径。详见：[PlatformIO in Cursor - OpenVSX](https://community.platformio.org/t/platformio-in-cursor-publication-to-openvsx/48602/5)。

### 配置文件（首次必做）

WiFi 凭证等本地配置放在 `include/config.h`，该文件已被 `.gitignore` 忽略、不会提交。首次克隆后需从模板生成：

```bash
cp include/config.example.h include/config.h
```

然后编辑 `include/config.h`，填入真实的 `WIFI_SSID`、`WIFI_PASS` 与 `AGENT_IP`。硬件参数（引脚、PID 等）已在模板中给出默认值，按实际硬件调整即可。

### 固件目录结构

```
WhiteBoardBot/
├── platformio.ini       # PlatformIO 配置（板型、框架、波特率等）
├── include/
│   └── config.example.h # 配置模板（复制为 config.h 后填入凭证）
└── src/
    └── main.cpp         # ESP32 固件入口
```

新增库依赖时，在 `platformio.ini` 的对应 `[env:xxx]` 下添加 `lib_deps = 库名`，或在 `lib/` 下放本地库。

---

## 四、部署说明

### 4.1 本地部署（开发机）

1. 克隆或拷贝项目到目标机器。
2. 安装 Python 3.9+。
3. 在项目根目录创建并激活虚拟环境（见第二节）。
4. 安装依赖：`pip install -r requirements.txt`。
5. 配置环境变量或配置文件（如有）。
6. 运行主程序或启动脚本。

### 4.2 生产/服务器部署要点

- **依赖固定**：建议在虚拟环境中生成并提交锁文件：
  ```bash
  pip freeze > requirements-lock.txt
  ```
  部署时使用：`pip install -r requirements-lock.txt` 以保证版本一致。

- **进程管理**：若需常驻运行，可使用：
  - **systemd**（Linux）
  - **supervisor**
  - 或 Docker（见下）

- **Docker 部署（可选）**

  在项目根目录创建 `Dockerfile` 示例：

  ```dockerfile
  FROM python:3.11-slim
  WORKDIR /app
  COPY requirements.txt .
  RUN pip install --no-cache-dir -r requirements.txt
  COPY . .
  CMD ["python", "-m", "src.main"]
  ```

  构建与运行：

  ```bash
  docker build -t whiteboardbot .
  docker run --rm -it whiteboardbot
  ```

  若需访问串口，请使用 `--device` 映射并注意权限。

### 4.3 硬件部署（ESP32）

1. **编译与烧录**：在项目根目录执行 `pio run -t upload`（或使用 PlatformIO 扩展的 Upload），USB 连接 ESP32 即可。
2. **串口监视**：`pio device monitor` 或扩展中的 Serial Monitor，波特率已在 `platformio.ini` 中设为 115200。
3. **Cursor 开发**：使用本项目自带的 PlatformIO 配置即可；与 MS C/C++ 扩展的兼容性说明见上文「3.4 PlatformIO」。
4. 部署后确保 USB 串口或无线连接与运行上位机（Python）的电脑可通信。

---

## 五、项目结构（当前）

```
WhiteBoardBot/
├── .venv/              # Python 虚拟环境（勿提交）
├── .gitignore
├── platformio.ini      # PlatformIO 配置（ESP32 板型、框架等）
├── requirements.txt    # Python 依赖
├── README.md           # 本使用与部署文档
└── src/
    └── main.cpp        # ESP32 固件入口（PlatformIO 编译）
```

- **Python 上位机**：使用 `.venv` + `requirements.txt`。
- **ESP32 固件**：使用 `platformio.ini` + `src/main.cpp`，通过 PlatformIO 编译与烧录。

---

## 六、常见问题

| 问题 | 处理 |
|------|------|
| `python3: command not found` | 安装 Python 3.9+，或使用 `py -3`（Windows） |
| 串口找不到或权限拒绝 | Linux 将用户加入 `dialout` 组；macOS 检查 USB 驱动 |
| Cursor 未使用 .venv | 用「Python: Select Interpreter」选择 `.venv` 下的 Python |
| Cursor 里找不到 PlatformIO | 从 OpenVSX 安装 VSIX，或用命令行 `pip install platformio` 后使用 `pio` 命令 |
| 烧录时找不到串口 | 插好 USB，执行 `pio device list` 查看；Windows 需装 CH340/CP210x 等驱动 |

---

*文档随项目更新，最后更新：2025-02-13*
