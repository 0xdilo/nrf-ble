//! Accept list (white list) filtering for advertising and scanning.

use crate::ll::addr::AddrType;

/// Maximum number of accept list entries.
pub const ACCEPT_LIST_MAX: usize = 8;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// A fixed-size accept list of (address, type) pairs.
pub struct AcceptList {
    entries: [(AddrType, [u8; 6]); ACCEPT_LIST_MAX],
    len: usize,
}

impl Default for AcceptList {
    fn default() -> Self {
        Self::new()
    }
}

impl AcceptList {
    /// Create an empty list.
    pub const fn new() -> Self {
        AcceptList {
            entries: [(AddrType::Public, [0; 6]); ACCEPT_LIST_MAX],
            len: 0,
        }
    }

    /// Remove all entries.
    pub fn clear(&mut self) {
        self.len = 0;
    }

    /// Add an entry; returns `false` when full or already present.
    pub fn add(&mut self, addr: [u8; 6], addr_type: AddrType) -> bool {
        if self.len >= ACCEPT_LIST_MAX || self.contains(addr, addr_type) {
            return false;
        }
        self.entries[self.len] = (addr_type, addr);
        self.len += 1;
        true
    }

    /// Number of entries.
    pub fn len(&self) -> usize {
        self.len
    }

    /// True when the list is empty.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// True when the (address, type) pair is in the list.
    pub fn contains(&self, addr: [u8; 6], addr_type: AddrType) -> bool {
        self.entries[..self.len]
            .iter()
            .any(|&(t, a)| t == addr_type && a == addr)
    }

    /// All entries as a slice.
    pub fn entries(&self) -> &[(AddrType, [u8; 6])] {
        &self.entries[..self.len]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn add_and_contains() {
        let mut list = AcceptList::new();
        assert!(list.add([1; 6], AddrType::Public));
        assert!(list.contains([1; 6], AddrType::Public));
        assert!(!list.contains([1; 6], AddrType::RandomStatic));
        assert!(!list.contains([2; 6], AddrType::Public));
    }

    #[test]
    fn duplicates_rejected() {
        let mut list = AcceptList::new();
        assert!(list.add([1; 6], AddrType::Public));
        assert!(!list.add([1; 6], AddrType::Public));
        assert_eq!(list.len(), 1);
    }

    #[test]
    fn capacity_limited() {
        let mut list = AcceptList::new();
        for i in 0..ACCEPT_LIST_MAX {
            assert!(list.add([i as u8; 6], AddrType::Public));
        }
        assert!(!list.add([0xFF; 6], AddrType::Public));
        assert_eq!(list.len(), ACCEPT_LIST_MAX);
    }

    #[test]
    fn clear_empties() {
        let mut list = AcceptList::new();
        list.add([1; 6], AddrType::Public);
        list.clear();
        assert!(list.is_empty());
    }
}
