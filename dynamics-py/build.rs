use std::process::Command;

pub fn main() {
    // Skip this build script when building documentation on docs.rs or when in CI
    if std::env::var("DOCS_RS").is_ok()
        || std::env::var("CI").is_ok()
        || std::env::var("GITHUB_ACTIONS").is_ok()
    {
        return;
    }

    if let Ok(var) = std::env::var("LINK_PYTHON_LIB")
        && var == "1"
    {
        // Get LIBDIR using sysconfig (Python 3.12+ compatible)
        let python_libdir = {
            let output = Command::new("python")
                .arg("-c")
                .arg("import sysconfig; print(sysconfig.get_config_var('LIBDIR'))")
                .output()
                .expect("Failed to get LIBDIR");
            if !output.status.success() {
                panic!("Failed to get LIBDIR");
            }
            String::from_utf8_lossy(&output.stdout).trim().to_string()
        };

        // Get LIBRARY (e.g., python3.13) using sysconfig
        let python_lib = {
            let output = Command::new("python")
                .arg("-c")
                .arg("import sysconfig; print(sysconfig.get_config_var('LDLIBRARY'))")
                .output()
                .expect("Failed to get LDLIBRARY");
            if !output.status.success() {
                panic!("Failed to get LDLIBRARY");
            }
            String::from_utf8_lossy(&output.stdout).trim().to_string()
        };

        // Remove 'lib' prefix and '.dylib' or '.a' suffix to get the correct name for rustc-link-lib
        let libname = python_lib
            .strip_prefix("lib")
            .unwrap_or(&python_lib)
            .trim_end_matches(".dylib")
            .trim_end_matches(".a");

        println!("cargo:rustc-link-search=native={python_libdir}");
        println!("cargo:rustc-link-lib=dylib={libname}");
        println!("cargo:rustc-link-arg=-Wl,-rpath,{python_libdir}");
    }
}
