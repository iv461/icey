# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

### Removed 

- Callback-based ServiceClient::call overload
- ServiceClient::cancel because it does not cancel the timeout timer correctly

### Added
 
- Support for actions
- Multi-threaded executor support for the async/await API

### Fixed

- Correct result type implementation
- Missing request cleanup in ServiceClientImpl::our_to_real_req_id_
- Added missing cancellation of timeout timer in Promise destruction

## 0.3.0 

### Fixed
- enables nested coroutines and with it fixes coroutine state memory leak

## 0.2.0 

### Changed

- Made Promise more efficient
- Separated async/await from reactive programming

## 0.1.0

Initial release 