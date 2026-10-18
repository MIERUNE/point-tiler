FROM docker.io/lukemathwalker/cargo-chef:0.1.78-rust-1.98.0-alpine3.24 AS chef
WORKDIR /app

FROM chef AS planner
COPY . .
RUN cargo chef prepare --recipe-path recipe.json

FROM chef AS builder

RUN rustup target add x86_64-unknown-linux-musl
RUN apk add --no-cache \
    g++ \
    cmake \
    make \
    pkgconf \
    proj \
    sqlite-dev \
    sqlite-static

COPY --from=planner /app/recipe.json recipe.json

RUN cargo chef cook \
    --release \
    --target x86_64-unknown-linux-musl \
    --recipe-path recipe.json

COPY . .
RUN cargo build \
    --release \
    --target x86_64-unknown-linux-musl \
    --bin ptiler

FROM gcr.io/distroless/static-debian13:nonroot AS runtime
WORKDIR /app

COPY --from=builder /app/target/x86_64-unknown-linux-musl/release/ptiler /ptiler

ENTRYPOINT ["/ptiler"]