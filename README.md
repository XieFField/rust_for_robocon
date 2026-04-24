# rust_for_robocon
只是个人的一次尝试，并不是GDUT在robocon赛场上所使用的代码

## 运行本仓库代码需要知道的

### 必须安装的组件
```
    rustup target add thumbv7em-none-eabihf
    rustup component add rust-src llvm-tools
    cargo install flip-link
    cargo install probe-rs-tools
    cargo install cargo-generate
```

### 运行时候的命令
```
    cargo build --release #编译
    probe-rs download --chip STM32H723ZG target\thumbv7em-none-eabihf\release\gdut-r1-by-rust #下载
    cargo run --release #也可以运行这一步
```
如果你是运行example中的内容
需要先执行`cargo clean` 再执行example的编译和下载
