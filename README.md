# Kiibohd Firmware

Rust successor to the [kiibohd-controller](https://github.com/kiibohd/controller) firmware.

Each keyboard has it's own directory.

* [common](common) - Common files used in all projects.
* [hexgears](hexgears) - Hexgears device firmware.
* [inputclub](inputclub) - Input Club device firmware.


## Top-Level Build Commands

These commands are useful when testing changes across multiple devices.

```bash
pipenv run cargo make check
pipenv run cargo make fmt
pipenv run cargo make fmt-check
pipenv run cargo make clippy

# Runs check, fmt-check and clippy
pipenv run cargo make sanity

# Builds gdb debug binaries for all crates
pipenv run cargo make build

# Builds release .dfu.bin for all crates
pipenv run cargo make bin-dfu
```

The commands are used by GitHub Actions to verify changes.


## License

Licensed under either of

 * Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.


### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any
additional terms or conditions.
