# rust_for_robocon
只是个人的一次尝试，并不是GDUT在robocon赛场上所使用的代码 -- XieFField

## 运行本仓库代码需要知道的

### 必须安装的组件
```
    rustup target add thumbv7em-none-eabihf
    rustup component add rust-src llvm-tools
    cargo install flip-link
    cargo install probe-rs-tools
    cargo install cargo-generate
```

我在使用正点原子的DAP-link时候发现，probe-rs会识别出两个调试器，即便他们的id完全相同。选择index0的会烧录不成功，只有选择index1才能烧录成功。但在launch文件中的probe指定索引却无法识别调试器。我索性把那个一模一样的id传进去，便能进去调试模式。
