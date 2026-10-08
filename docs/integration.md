# Integrating Chapter 4 into IET_code

The [book repository](https://github.com/PolyU-TASLAB/IET_code) already organizes
`Chapter1` and `Chapter5` as Git submodules. Its maintainer can add this repository
as `Chapter4`. Both repositories currently require private-repository access;
access to the book repository does not grant access to this chapter repository.

## Initial integration / 首次接入

Run in a local checkout of `IET_code`, on the integration branch selected by its
maintainer. The following commands add only Chapter 4:

在总仓库的本地目录执行，将第 4 章接入为子模块，并跟踪 `main`：

```bash
git submodule add -b main https://github.com/JIAHAO-FUHUA/uav-planning-book-examples.git Chapter4
git add .gitmodules Chapter4
git commit -m "Add Chapter 4 UAV planning examples"
git push
```

## Author updates / 作者更新代码

Run in the standalone `uav-planning-book-examples` checkout after editing:

你在自己的代码仓库更新，再提交并推送；同学即可在章节仓库看到新版：

```bash
git add examples docs README.md README_ZH.md requirements.txt CMakeLists.txt tools .github
git commit -m "Update UAV planning examples"
git push origin main
```

## Adopt the latest chapter / 总仓库采用新版

Run from the `IET_code` root after the author pushes:

同学在总仓库更新第 4 章，并提交总仓库记录的版本：

```bash
git submodule update --init --remote -- Chapter4
git add Chapter4
git commit -m "Update Chapter 4 to the latest examples"
git push
```

The book repository pins a chapter commit. Tracking `main` lets `--remote` obtain
its latest commit, but the maintainer must still commit the updated chapter pointer.

子模块记录具体提交。作者推送后，总仓库不会自动改变；同学执行上述更新并推送，
整合版本才会采用新版。这样每次书稿发布都能对应确定的代码版本。

## Readers / 其他读者

After cloning or pulling `IET_code`, obtain its recorded Chapter 4 version with:

克隆或拉取总仓库后，下载其已记录的第 4 章版本：

```bash
git submodule update --init -- Chapter4
```

References: [Git submodule manual](https://git-scm.com/docs/git-submodule).
