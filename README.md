# Embers

Embers is a collection of helpful drivers to use in embedded Rust, targeting easy to use implementations for important protocols and sensor modules. The name Embers comes from EMBedded drivERS!

It uses the embedded-io-async implementations, so these drivers can be used with most available embedded Rust frameworks!

# Usage

Embers is just like any other crate, and you can add it with cargo:

```rs
cargo add embers
```

The drivers are split into subfolders in the `src/` directory, and each subfolder will contain a README.md to show an example of using the driver (usually with Embassy).
