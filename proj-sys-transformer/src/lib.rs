use std::{
    ffi::{CStr, CString},
    os::raw::c_char,
    path::Path,
    ptr,
};

use pcd_core::pointcloud::point::Point;
use proj_sys as proj;

pub type EpsgCode = u16;

#[derive(Debug)]
pub struct ProjError {
    pub code: i32,
    pub message: String,
    pub context: &'static str,
}

impl std::fmt::Display for ProjError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "PROJ error ({}): {} {}",
            self.context, self.code, self.message
        )
    }
}

impl std::error::Error for ProjError {}

#[derive(Debug)]
pub struct ProjTransformer {
    ctx: *mut proj::PJ_CONTEXT,
    pj: *mut proj::PJ,
}

impl ProjTransformer {
    pub fn new_epsg(
        input_epsg: EpsgCode,
        output_epsg: EpsgCode,
        proj_data_dir: Option<&Path>,
    ) -> Result<Self, ProjError> {
        let source = format!("EPSG:{input_epsg}");
        let target = format!("EPSG:{output_epsg}");
        Self::new(&source, &target, proj_data_dir)
    }

    pub fn new(
        source: &str,
        target: &str,
        proj_data_dir: Option<&Path>,
    ) -> Result<Self, ProjError> {
        // Context is recommended for multi-threaded use; a context must be used by
        // only one thread at a time.
        let ctx = unsafe { proj::proj_context_create() };
        if ctx.is_null() {
            return Err(ProjError {
                code: 0,
                message: "proj_context_create() returned NULL".to_string(),
                context: "proj_context_create",
            });
        }

        // Enable network access for automatic grid file downloads from CDN.
        // Grid files will be cached in the user's local directory.
        unsafe {
            proj::proj_context_set_enable_network(ctx, 1);
            proj::proj_grid_cache_set_enable(ctx, 1);
        }

        if let Some(dir) = proj_data_dir {
            let c_path = CString::new(dir.to_string_lossy().as_bytes()).map_err(|_| ProjError {
                code: 0,
                message: "proj_data_dir contains NUL byte".to_string(),
                context: "proj_context_set_search_paths",
            })?;
            let paths = [c_path.as_ptr()];
            unsafe {
                proj::proj_context_set_search_paths(ctx, paths.len() as i32, paths.as_ptr());
            }
        }

        let source = CString::new(source).map_err(|_| ProjError {
            code: 0,
            message: "source CRS contains NUL byte".to_string(),
            context: "proj_create_crs_to_crs",
        })?;
        let target = CString::new(target).map_err(|_| ProjError {
            code: 0,
            message: "target CRS contains NUL byte".to_string(),
            context: "proj_create_crs_to_crs",
        })?;

        let pj = unsafe {
            proj::proj_create_crs_to_crs(ctx, source.as_ptr(), target.as_ptr(), ptr::null_mut())
        };
        if pj.is_null() {
            let err = proj_error_from_ctx(ctx, "proj_create_crs_to_crs");
            unsafe {
                proj::proj_context_destroy(ctx);
            }
            return Err(err);
        }

        // Normalize axis order (e.g. EPSG:4326 is lat,lon by definition).
        let normalized = unsafe { proj::proj_normalize_for_visualization(ctx, pj) };
        unsafe {
            proj::proj_destroy(pj);
        }
        if normalized.is_null() {
            let err = proj_error_from_ctx(ctx, "proj_normalize_for_visualization");
            unsafe {
                proj::proj_context_destroy(ctx);
            }
            return Err(err);
        }

        Ok(Self {
            ctx,
            pj: normalized,
        })
    }

    pub fn transform_points_in_place(&mut self, points: &mut [Point]) -> Result<(), ProjError> {
        if points.is_empty() {
            return Ok(());
        }

        let stride = std::mem::size_of::<Point>();
        let n = points.len();

        unsafe {
            proj::proj_errno_reset(self.pj);

            let first = points.as_mut_ptr();
            let x = ptr::addr_of_mut!((*first).x);
            let y = ptr::addr_of_mut!((*first).y);
            let z = ptr::addr_of_mut!((*first).z);

            proj::proj_trans_generic(
                self.pj,
                proj::PJ_DIRECTION_PJ_FWD,
                x,
                stride,
                n,
                y,
                stride,
                n,
                z,
                stride,
                n,
                ptr::null_mut(),
                0,
                0,
            );

            let err = proj::proj_errno(self.pj);
            if err != 0 {
                return Err(proj_error_from_pj(self.ctx, self.pj, "proj_trans_generic"));
            }
        }

        Ok(())
    }
}

impl Drop for ProjTransformer {
    fn drop(&mut self) {
        unsafe {
            if !self.pj.is_null() {
                proj::proj_destroy(self.pj);
                self.pj = ptr::null_mut();
            }
            if !self.ctx.is_null() {
                proj::proj_context_destroy(self.ctx);
                self.ctx = ptr::null_mut();
            }
        }
    }
}

fn proj_error_from_ctx(ctx: *mut proj::PJ_CONTEXT, context: &'static str) -> ProjError {
    let code = unsafe { proj::proj_context_errno(ctx) } as i32;
    let message = proj_error_message(ctx, code);
    ProjError {
        code,
        message,
        context,
    }
}

fn proj_error_from_pj(
    ctx: *mut proj::PJ_CONTEXT,
    pj: *mut proj::PJ,
    context: &'static str,
) -> ProjError {
    let code = unsafe { proj::proj_errno(pj) } as i32;
    let message = proj_error_message(ctx, code);
    ProjError {
        code,
        message,
        context,
    }
}

fn proj_error_message(ctx: *mut proj::PJ_CONTEXT, code: i32) -> String {
    let c_msg = unsafe { proj::proj_context_errno_string(ctx, code as i32) };
    if c_msg.is_null() {
        return "unknown error".to_string();
    }
    unsafe { CStr::from_ptr(c_msg as *const c_char) }
        .to_string_lossy()
        .into_owned()
}
