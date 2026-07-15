-- SPDX-FileCopyrightText: 2026 GMaster contributors
-- SPDX-License-Identifier: Apache-2.0

local source_path = debug.getinfo(1, "S").source:gsub("^@", "")
local project_root = vim.fs.dirname(vim.fs.normalize(vim.fn.fnamemodify(source_path, ":p")))
local west_marker = vim.fs.find(".west", {
  path = project_root,
  upward = true,
  type = "directory",
})[1]
local west_topdir = west_marker and vim.fs.dirname(west_marker) or nil

local function normalize_path(path)
  return vim.fs.normalize(vim.fn.fnamemodify(vim.fn.expand(path), ":p"))
end

local function project_path(path)
  local expanded = vim.fn.expand(path)

  if expanded:match("^/") or expanded:match("^%a:[/\\]") then
    return normalize_path(expanded)
  end

  return normalize_path(project_root .. "/" .. expanded)
end

local function is_within(path, root)
  local normalized_path = normalize_path(path)
  local normalized_root = normalize_path(root):gsub("/$", "")

  return normalized_path == normalized_root
    or normalized_path:sub(1, #normalized_root + 1) == normalized_root .. "/"
end

local function read_cmake_cache(build_dir)
  local cache = {}
  local file = io.open(build_dir .. "/CMakeCache.txt", "r")

  if not file then
    return cache
  end

  for line in file:lines() do
    local key, value = line:match("^([^:#][^:]*):[^=]+=(.*)$")

    if key then
      cache[key] = value
    end
  end

  file:close()
  return cache
end

local function inspect_build_dir(build_dir)
  local normalized_dir = normalize_path(build_dir)
  local compile_commands = normalized_dir .. "/compile_commands.json"

  if vim.fn.filereadable(compile_commands) ~= 1 then
    return nil
  end

  local cache = read_cmake_cache(normalized_dir)
  local application_source = cache.APPLICATION_SOURCE_DIR
  local zephyr_base = cache.ZEPHYR_BASE

  if application_source and not is_within(application_source, project_root) then
    return nil
  end

  if west_topdir and zephyr_base and not is_within(zephyr_base, west_topdir) then
    return nil
  end

  local stat = vim.uv.fs_stat(compile_commands)

  return {
    dir = normalized_dir,
    cache = cache,
    modified = stat and stat.mtime.sec or 0,
  }
end

local function find_build_dir()
  local override = vim.env.GZRL_CLANGD_BUILD_DIR

  if override and override ~= "" then
    local build = inspect_build_dir(project_path(override))

    if build then
      return build
    end

    vim.schedule(function()
      vim.notify(
        "GZRL_CLANGD_BUILD_DIR does not contain a valid GZRL compile database: " .. override,
        vim.log.levels.WARN
      )
    end)
  end

  local builds = {}
  local seen = {}
  local databases = vim.fn.globpath(project_root, "build*/**/compile_commands.json", false, true)

  for _, database in ipairs(databases) do
    local build_dir = normalize_path(vim.fs.dirname(database))

    if not seen[build_dir] then
      seen[build_dir] = true
      local build = inspect_build_dir(build_dir)

      if build then
        table.insert(builds, build)
      end
    end
  end

  table.sort(builds, function(left, right)
    if left.modified == right.modified then
      return left.dir < right.dir
    end

    return left.modified > right.modified
  end)

  return builds[1]
end

local function clangd_command()
  local command = {
    "clangd",
    "--background-index",
    "--clang-tidy",
    "--header-insertion=iwyu",
    "--completion-style=detailed",
    "--function-arg-placeholders",
    "--fallback-style=llvm",
    "--pch-storage=memory",
  }
  local build = find_build_dir()

  if not build then
    vim.schedule(function()
      vim.notify(
        "No GZRL compile database found; build a target and run :LspRestart",
        vim.log.levels.WARN
      )
    end)
    return command
  end

  table.insert(command, "--compile-commands-dir=" .. build.dir)

  local drivers = {}
  local seen = {}

  for _, key in ipairs({ "CMAKE_C_COMPILER", "CMAKE_CXX_COMPILER" }) do
    local driver = build.cache[key]

    if driver and driver ~= "" then
      driver = normalize_path(driver)

      if not seen[driver] then
        seen[driver] = true
        table.insert(drivers, driver)
      end
    end
  end

  if #drivers > 0 then
    table.insert(command, "--query-driver=" .. table.concat(drivers, ","))
  end

  return command
end

return {
  {
    "neovim/nvim-lspconfig",
    opts = {
      servers = {
        clangd = {
          cmd = clangd_command(),
          on_new_config = function(config)
            config.cmd = clangd_command()
          end,
        },
      },
      diagnostics = {
        virtual_text = {
          severity = { min = vim.diagnostic.severity.ERROR },
        },
      },
    },
  },
  {
    "stevearc/conform.nvim",
    opts = {
      formatters_by_ft = {
        c = { "clang_format" },
        cpp = { "clang_format" },
      },
    },
  },
}
