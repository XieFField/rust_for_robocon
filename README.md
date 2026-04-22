# rust_for_robocon
尝试在stm32上使用rust控制robocon机器人，并尝试用rust重构先前的cpp框架
只是个人的一次尝试，并不是GDUT在robocon赛场上所使用的代码

## 运行本仓库代码的必须组件
```
    rustup target add thumbv7em-none-eabihf
    rustup component add rust-src llvm-tools
    cargo install flip-link
    cargo install probe-rs-tools
    cargo install cargo-generate
```